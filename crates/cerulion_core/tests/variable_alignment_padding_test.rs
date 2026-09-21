// SPDX-License-Identifier: AGPL-3.0-only
//! Generated primitive-array writers must initialize alignment gaps inside the
//! published payload. These use the production writer over poisoned, aligned
//! buffers: no shared memory, hardware, or generated-source edits are needed.

use cerulion_core::message::ShmMessage;
use cerulion_core::wire::MaxPayloadCapacity;
use cerulion_core::TransportError;
use native_ros2_messages::sensor_msgs::{JointState, LaserScan};
use std::sync::Arc;

#[repr(align(8))]
struct Buffer([u8; 256]);

// A valid Header: zero stamp; frame_id at byte 16, length 1, value "x".
const HEADER: [u8; 17] = [0, 0, 0, 0, 0, 0, 0, 0, 16, 0, 0, 0, 1, 0, 0, 0, b'x'];

#[derive(Clone, Copy, Debug)]
enum WritePath {
    Loan,
    Set,
    Push,
    Fill,
}

const PATHS: [WritePath; 4] = [
    WritePath::Loan,
    WritePath::Set,
    WritePath::Push,
    WritePath::Fill,
];

fn scan_oracle() -> [u8; 76] {
    // Seven fixed f32s, three offset entries, the Header, three zero padding
    // bytes, and one f32 (1.25). The empty intensities field ends at byte 76.
    let mut expected = [0; 76];
    expected[28..52].copy_from_slice(&[
        52, 0, 0, 0, 17, 0, 0, 0, 72, 0, 0, 0, 4, 0, 0, 0, 76, 0, 0, 0, 0, 0, 0, 0,
    ]);
    expected[52..69].copy_from_slice(&HEADER);
    expected[72..76].copy_from_slice(&[0, 0, 160, 63]);
    expected
}

#[test]
fn all_f32_write_paths_clear_only_the_published_alignment_gap() {
    for path in PATHS {
        for poison in [0xa5, 0x5a] {
            let mut buffer = Buffer([poison; 256]);
            // Match publisher initialization: fixed fields and table are zero,
            // while the variable region retains the slot's previous contents.
            buffer.0[..52].fill(0);
            {
                let mut writer = LaserScan::build_writer(
                    &mut buffer.0,
                    MaxPayloadCapacity::const_new(256),
                    Arc::from("padding/scan"),
                );
                writer.set_header_bytes(&HEADER).unwrap();
                match path {
                    WritePath::Loan => writer.loan_ranges(1).unwrap()[0] = 1.25,
                    WritePath::Set => writer.set_ranges(&[1.25]).unwrap(),
                    WritePath::Push => writer.push_ranges(1.25).unwrap(),
                    WritePath::Fill => writer
                        .fill_from_ranges(|dst: &mut [f32]| {
                            dst[0] = 1.25;
                            Ok(1)
                        })
                        .unwrap(),
                }
                writer.set_intensities(&[]).unwrap();
                assert_eq!(writer.ranges(), [1.25]);
                assert!(LaserScan::all_variables_written(&writer));
                assert_eq!(LaserScan::payload_wire_size(&writer), 76);
                assert!(!writer.has_overflow());
            }
            assert_eq!(&buffer.0[..76], &scan_oracle(), "{path:?}, {poison:#x}");
            assert!(buffer.0[76..].iter().all(|&byte| byte == poison));
        }
    }
}

#[test]
fn eight_byte_elements_clear_the_full_seven_byte_gap() {
    for path in PATHS {
        let mut buffer = Buffer([0xa5; 256]);
        {
            let mut writer = JointState::build_writer(
                &mut buffer.0,
                MaxPayloadCapacity::const_new(256),
                Arc::from("padding/joints"),
            );
            writer.set_header_bytes(&HEADER).unwrap();
            writer.set_name_bytes(&[]).unwrap();
            match path {
                WritePath::Loan => writer.loan_position(1).unwrap()[0] = 2.5,
                WritePath::Set => writer.set_position(&[2.5]).unwrap(),
                WritePath::Push => writer.push_position(2.5).unwrap(),
                WritePath::Fill => writer
                    .fill_from_position(|dst: &mut [f64]| {
                        dst[0] = 2.5;
                        Ok(1)
                    })
                    .unwrap(),
            }
            writer.set_velocity(&[]).unwrap();
            writer.set_effort(&[]).unwrap();
            assert!(JointState::all_variables_written(&writer));
            assert_eq!(JointState::payload_wire_size(&writer), 72);
        }
        let mut expected = [0; 72];
        expected[..40].copy_from_slice(&[
            40, 0, 0, 0, 17, 0, 0, 0, 57, 0, 0, 0, 0, 0, 0, 0, 64, 0, 0, 0, 8, 0, 0, 0, 72, 0, 0,
            0, 0, 0, 0, 0, 72, 0, 0, 0, 0, 0, 0, 0,
        ]);
        expected[40..57].copy_from_slice(&HEADER);
        expected[64..72].copy_from_slice(&[0, 0, 0, 0, 0, 0, 4, 64]);
        assert_eq!(&buffer.0[..72], &expected, "{path:?}");
        assert!(buffer.0[72..].iter().all(|&byte| byte == 0xa5));
    }
}

#[test]
fn zero_length_arrays_still_initialize_their_committed_padding() {
    for fill in [false, true] {
        let mut buffer = Buffer([0xa5; 256]);
        buffer.0[..52].fill(0);
        {
            let mut writer = LaserScan::build_writer(
                &mut buffer.0,
                MaxPayloadCapacity::const_new(256),
                Arc::from("padding/empty"),
            );
            writer.set_header_bytes(&HEADER).unwrap();
            if fill {
                writer.fill_from_ranges(|_: &mut [f32]| Ok(0)).unwrap();
            } else {
                assert!(writer.loan_ranges(0).unwrap().is_empty());
            }
            writer.set_intensities(&[]).unwrap();
            assert_eq!(LaserScan::payload_wire_size(&writer), 72);
            assert!(LaserScan::all_variables_written(&writer));
        }
        assert_eq!(&buffer.0[69..72], &[0; 3]);
        assert!(buffer.0[72..].iter().all(|&byte| byte == 0xa5));
    }
}

#[test]
fn alignment_beyond_the_original_buffer_is_cleared_after_spilling() {
    for path in PATHS {
        let mut buffer = Buffer([0xa5; 256]);
        buffer.0[..52].fill(0);
        let mut writer = LaserScan::build_writer(
            &mut buffer.0[..70],
            MaxPayloadCapacity::const_new(256),
            Arc::from("padding/spill"),
        );
        writer.set_header_bytes(&HEADER).unwrap();
        match path {
            WritePath::Loan => writer.loan_ranges(1).unwrap()[0] = 1.25,
            WritePath::Set => writer.set_ranges(&[1.25]).unwrap(),
            WritePath::Push => writer.push_ranges(1.25).unwrap(),
            WritePath::Fill => writer
                .fill_from_ranges(|dst: &mut [f32]| {
                    dst[0] = 1.25;
                    Ok(1)
                })
                .unwrap(),
        }
        writer.set_intensities(&[]).unwrap();
        assert_eq!(writer.overflow_view_bytes(), Some(scan_oracle().as_slice()));
    }
}

#[test]
fn rejected_capacity_does_not_commit_or_clear_an_alignment_gap() {
    for path in PATHS {
        let mut buffer = Buffer([0xa5; 256]);
        buffer.0[..52].fill(0);
        {
            let mut writer = LaserScan::build_writer(
                &mut buffer.0[..70],
                MaxPayloadCapacity::const_new(70),
                Arc::from("padding/ceiling"),
            );
            writer.set_header_bytes(&HEADER).unwrap();
            let result = match path {
                WritePath::Loan => writer.loan_ranges(1).map(|_| ()),
                WritePath::Set => writer.set_ranges(&[1.25]),
                WritePath::Push => writer.push_ranges(1.25),
                WritePath::Fill => writer.fill_from_ranges(|_: &mut [f32]| {
                    panic!("producer must not run when alignment exceeds the ceiling")
                }),
            };
            assert!(matches!(
                result,
                Err(TransportError::PayloadTooLarge { .. })
            ));
            assert_eq!(LaserScan::payload_wire_size(&writer), 69);
            assert!(!LaserScan::all_variables_written(&writer));
        }
        assert_eq!(&buffer.0[36..52], &[0; 16], "failed fields stay unwritten");
        assert_eq!(&buffer.0[52..69], &HEADER);
        assert!(buffer.0[69..].iter().all(|&byte| byte == 0xa5));
    }
}

#[test]
fn failed_or_panicking_producer_does_not_commit_alignment_padding() {
    for panic in [false, true] {
        let mut buffer = Buffer([0xa5; 256]);
        buffer.0[..52].fill(0);
        {
            let mut writer = LaserScan::build_writer(
                &mut buffer.0,
                MaxPayloadCapacity::const_new(256),
                Arc::from("padding/producer-failure"),
            );
            writer.set_header_bytes(&HEADER).unwrap();
            let result = std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| {
                writer.fill_from_ranges(|dst: &mut [f32]| {
                    dst[0] = 1.25;
                    if panic {
                        panic!("producer failure");
                    }
                    Err(TransportError::NodeError {
                        node_id: "test".into(),
                        reason: "producer failure".into(),
                    })
                })
            }));
            if panic {
                assert!(result.is_err());
            } else {
                assert!(matches!(result, Ok(Err(TransportError::NodeError { .. }))));
            }
            assert_eq!(LaserScan::payload_wire_size(&writer), 69);
            assert!(!LaserScan::all_variables_written(&writer));
        }
        assert_eq!(&buffer.0[36..52], &[0; 16]);
        assert_eq!(&buffer.0[52..69], &HEADER);
        assert_eq!(
            &buffer.0[69..72],
            &[0xa5; 3],
            "gap is outside committed bytes"
        );
    }
}
