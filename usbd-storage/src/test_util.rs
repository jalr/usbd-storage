//! Test helpers: a mock [UsbBus] that supports control transfers on EP0, so
//! classes can be driven through a real [usb_device::device::UsbDevice].

use std::collections::{HashSet, VecDeque};
use std::sync::{Arc, Mutex};
use usb_device::bus::{PollResult, UsbBus, UsbBusAllocator};
use usb_device::class_prelude::{EndpointAddress, EndpointType};
use usb_device::{UsbDirection, UsbError};

#[derive(Default)]
pub struct Inner {
    /// SETUP packets the host sends on EP0
    pub setup: VecDeque<[u8; 8]>,
    /// Bulk OUT packets the host sends, per endpoint index
    pub out: [VecDeque<Vec<u8>>; 16],
    /// Packets the device has written, with the endpoint they went to
    pub written: Vec<(EndpointAddress, Vec<u8>)>,
    /// Currently stalled endpoints
    pub stalled: HashSet<u8>,
    next_ep: [u8; 2],
}

#[derive(Clone, Default)]
pub struct Shared(pub Arc<Mutex<Inner>>);

impl Shared {
    pub fn setup(&self, packet: [u8; 8]) {
        self.0.lock().unwrap().setup.push_back(packet);
    }

    pub fn bulk_out(&self, ep: EndpointAddress, bytes: &[u8], packet_size: usize) {
        let mut inner = self.0.lock().unwrap();
        for chunk in bytes.chunks(packet_size) {
            inner.out[ep.index()].push_back(chunk.to_vec());
        }
    }

    pub fn take_written(&self, ep: EndpointAddress) -> Vec<Vec<u8>> {
        let mut inner = self.0.lock().unwrap();
        let (taken, kept) = std::mem::take(&mut inner.written)
            .into_iter()
            .partition(|(addr, _)| *addr == ep);
        inner.written = kept;
        taken.into_iter().map(|(_, data)| data).collect()
    }

    pub fn is_stalled(&self, ep: EndpointAddress) -> bool {
        self.0.lock().unwrap().stalled.contains(&u8::from(ep))
    }
}

pub struct MockBus(pub Shared);

impl UsbBus for MockBus {
    fn alloc_ep(
        &mut self,
        ep_dir: UsbDirection,
        ep_addr: Option<EndpointAddress>,
        _ep_type: EndpointType,
        _max_packet_size: u16,
        _interval: u8,
    ) -> usb_device::Result<EndpointAddress> {
        if let Some(addr) = ep_addr {
            return Ok(addr);
        }
        let mut inner = self.0.0.lock().unwrap();
        let next = &mut inner.next_ep[ep_dir as usize >> 7];
        *next += 1;
        Ok(EndpointAddress::from_parts(*next as usize, ep_dir))
    }

    fn enable(&mut self) {}
    fn reset(&self) {}
    fn set_device_address(&self, _addr: u8) {}

    fn write(&self, ep_addr: EndpointAddress, buf: &[u8]) -> usb_device::Result<usize> {
        self.0
            .0
            .lock()
            .unwrap()
            .written
            .push((ep_addr, buf.to_vec()));
        Ok(buf.len())
    }

    fn read(&self, ep_addr: EndpointAddress, buf: &mut [u8]) -> usb_device::Result<usize> {
        let mut inner = self.0.0.lock().unwrap();
        let pkt = if ep_addr.index() == 0 {
            inner.setup.pop_front().map(|s| s.to_vec())
        } else {
            inner.out[ep_addr.index()].pop_front()
        };
        match pkt {
            Some(pkt) => {
                buf[..pkt.len()].copy_from_slice(&pkt);
                Ok(pkt.len())
            }
            None => Err(UsbError::WouldBlock),
        }
    }

    fn set_stalled(&self, ep_addr: EndpointAddress, stalled: bool) {
        let mut inner = self.0.0.lock().unwrap();
        if stalled {
            inner.stalled.insert(u8::from(ep_addr));
        } else {
            inner.stalled.remove(&u8::from(ep_addr));
        }
    }

    fn is_stalled(&self, ep_addr: EndpointAddress) -> bool {
        self.0
            .0
            .lock()
            .unwrap()
            .stalled
            .contains(&u8::from(ep_addr))
    }

    fn suspend(&self) {}
    fn resume(&self) {}

    fn poll(&self) -> PollResult {
        let inner = self.0.0.lock().unwrap();
        let ep_setup = if inner.setup.is_empty() { 0 } else { 1 };
        let ep_out = inner
            .out
            .iter()
            .enumerate()
            .filter(|(_, q)| !q.is_empty())
            .fold(0u16, |bits, (i, _)| bits | 1 << i);
        if ep_setup == 0 && ep_out == 0 {
            PollResult::None
        } else {
            PollResult::Data {
                ep_out,
                ep_in_complete: 0,
                ep_setup,
            }
        }
    }
}

pub fn new_alloc() -> (Shared, &'static UsbBusAllocator<MockBus>) {
    let shared = Shared::default();
    let alloc = Box::leak(Box::new(UsbBusAllocator::new(MockBus(shared.clone()))));
    (shared, alloc)
}

pub const EP0_IN: u8 = 0x80;
pub const BULK_OUT: u8 = 0x01;
pub const BULK_IN: u8 = 0x81;

/// A 31-byte CBW for a no-data command with the given CB.
pub fn cbw_no_data(cb: &[u8]) -> Vec<u8> {
    let mut c = vec![0u8; 31];
    c[..4].copy_from_slice(&0x43425355u32.to_le_bytes());
    c[4..8].copy_from_slice(&1u32.to_le_bytes()); // tag
    c[14] = cb.len() as u8;
    c[15..15 + cb.len()].copy_from_slice(cb);
    c
}

/// Bulk-Only Mass Storage Reset (USB MSC BOT, section 3.1) for `interface`.
pub fn bomsr_setup(interface: u8) -> [u8; 8] {
    [0x21, 0xFF, 0, 0, interface, 0, 0, 0]
}

/// CLEAR_FEATURE(ENDPOINT_HALT) for `ep`.
pub fn clear_halt_setup(ep: u8) -> [u8; 8] {
    [0x02, 0x01, 0, 0, ep, 0, 0, 0]
}

/// Drives a class through an invalid CBW followed by the Reset Recovery a
/// host performs (Spec. 5.3.4): Bulk-Only Mass Storage Reset, then
/// CLEAR_FEATURE(ENDPOINT_HALT) on both bulk endpoints. Afterwards the class
/// must accept the next CBW.
pub fn assert_reset_recovery_works<C: usb_device::class::UsbClass<MockBus>>(
    shared: &Shared,
    alloc: &'static UsbBusAllocator<MockBus>,
    class: &mut C,
    has_command: impl Fn(&C) -> bool,
) {
    use usb_device::device::{UsbDeviceBuilder, UsbVidPid};

    let mut dev = UsbDeviceBuilder::new(alloc, UsbVidPid(0xabcd, 0xabcd)).build();
    let ep = |addr: u8| EndpointAddress::from(addr);

    // An invalid CBW: the device must STALL both bulk endpoints (Spec. 6.6.1)
    shared.bulk_out(ep(BULK_OUT), &[0u8; 31], 64);
    for _ in 0..4 {
        dev.poll(&mut [&mut *class]);
    }
    assert!(!has_command(class));
    assert!(shared.is_stalled(ep(BULK_IN)) && shared.is_stalled(ep(BULK_OUT)));

    // Bulk-Only Mass Storage Reset: must be ACKed with a zero-length status stage
    shared.take_written(ep(EP0_IN));
    shared.setup(bomsr_setup(0));
    dev.poll(&mut [&mut *class]);
    assert_eq!(
        shared.take_written(ep(EP0_IN)),
        vec![Vec::<u8>::new()],
        "Bulk-Only Mass Storage Reset was not accepted"
    );
    assert!(!shared.is_stalled(ep(EP0_IN)) && !shared.is_stalled(ep(0x00)));
    // Spec. 3.1: the reset preserves the STALL conditions
    assert!(shared.is_stalled(ep(BULK_IN)) && shared.is_stalled(ep(BULK_OUT)));

    // The host clears the halts
    for addr in [BULK_IN, BULK_OUT] {
        shared.setup(clear_halt_setup(addr));
        dev.poll(&mut [&mut *class]);
    }
    assert!(!shared.is_stalled(ep(BULK_IN)) && !shared.is_stalled(ep(BULK_OUT)));

    // The device accepts the next CBW (TEST UNIT READY)
    shared.bulk_out(ep(BULK_OUT), &cbw_no_data(&[0u8; 6]), 64);
    for _ in 0..4 {
        dev.poll(&mut [&mut *class]);
    }
    assert!(has_command(class), "device did not recover after the reset");
}

#[cfg(test)]
mod tests {
    use super::{assert_reset_recovery_works, new_alloc};

    #[cfg(feature = "scsi")]
    #[test]
    fn scsi_bulk_only_mass_storage_reset_recovers_from_invalid_cbw() {
        use crate::subclass::scsi::Scsi;

        let (shared, alloc) = new_alloc();
        let mut scsi = Scsi::new(alloc, 64, 0, vec![0u8; 512]).unwrap();
        assert_reset_recovery_works(&shared, alloc, &mut scsi, |c| {
            c.transport.get_command().is_some()
        });
    }

    #[cfg(feature = "ufi")]
    #[test]
    fn ufi_bulk_only_mass_storage_reset_recovers_from_invalid_cbw() {
        use crate::subclass::ufi::Ufi;

        let (shared, alloc) = new_alloc();
        let mut ufi = Ufi::new(alloc, 64, vec![0u8; 512]).unwrap();
        assert_reset_recovery_works(&shared, alloc, &mut ufi, |c| {
            c.transport.get_command().is_some()
        });
    }
}
