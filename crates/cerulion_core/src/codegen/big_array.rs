// SPDX-License-Identifier: AGPL-3.0-only
//! Serde support for codegen-emitted fixed arrays longer than 32 elements.
//!
//! # Why this exists
//!
//! `serde` implements `Serialize`/`Deserialize` for `[T; N]` only up to
//! `N = 32` (verified against serde 1.0.229: `[f64; 36]` fails to derive with
//! "the trait `Deserialize<'de>` is not implemented for `[f64; 36]`"). Three
//! vendored ROS 2 messages carry a `float64[36]` covariance matrix
//! (`geometry_msgs/{Pose,Twist,Accel}WithCovariance`), and since nested
//! resolution became transitive that absence poisoned every schema embedding
//! one — nine `<Name>Snapshot` types in total, including `nav_msgs/Odometry`,
//! which is among the most commonly held pieces of robot node state.
//!
//! Codegen emits `#[serde(with = "::cerulion_core::codegen::big_array")]` on
//! any snapshot field whose type is a fixed array longer than serde's blanket
//! ceiling, so the whole generated corpus derives serde uniformly.
//!
//! # Why not `serde_big_array`
//!
//! Capability is identical (that crate has the same `[T; N]`-with-a-
//! `Serialize` element shape and the same non-recursive limit), so the choice
//! is about surface: routing the generated `#[serde(with = ...)]` path through
//! a third-party crate would require re-exporting it from `cerulion_core`'s
//! public API — a second permanent `pub use` of somebody else's crate — plus a
//! new workspace dependency and a `cargo-deny` entry, to replace ~40 lines
//! that are oracle-tested here.
//!
//! # Encoding
//!
//! Identical in shape to serde's own array impls: a fixed-length TUPLE, not a
//! sequence. Self-describing formats (JSON) see an array either way, while
//! compact formats (bincode, postcard) omit the length prefix a `seq` would
//! add — so a hand-written 32-element sibling and a 36-element field encode
//! the same way, and swapping this helper in or out of a `[T; 32]` field would
//! not change its bytes.
//!
//! # Limit (deliberate, and unreachable from every front end)
//!
//! Elements must themselves be `Serialize`/`Deserialize`, so a fixed array OF
//! large fixed arrays (`[[f64; 36]; 4]`) is not supported: the outer array is
//! within serde's ceiling while the inner is not, and this helper cannot reach
//! it. No schema front end can express that shape — both `.msg`
//! ([`crate::codegen::parse_rosmsg`]) and the YAML type parser find the FIRST
//! `[` and parse everything after it as one length, so `float64[4][36]` is a
//! parse error, not a nested array. Such a schema is only constructible by
//! building [`crate::codegen::MessageSchema`] by hand, and the failure mode is
//! a loud compile error in the generated code rather than a silent mis-encode.

use core::fmt;
use core::marker::PhantomData;

use serde::de::{self, SeqAccess, Visitor};
use serde::ser::SerializeTuple;
use serde::{Deserialize, Deserializer, Serialize, Serializer};

/// Largest `N` for which `serde` implements `Serialize`/`Deserialize` on
/// `[T; N]` without help (serde 1.0.229: `array_impls! { 1 2 ... 32 }`).
///
/// Defined HERE, beside the helper that exists because of it, and read by
/// [`crate::codegen::generate_schema`] to decide which snapshot fields get a
/// `#[serde(with = ...)]` — so the number the generator keys on and the number
/// this module's tests probe cannot drift apart.
pub const SERDE_ARRAY_IMPL_CEILING: usize = 32;

/// Serialize `[T; N]` as a fixed-length tuple, for any `N`.
///
/// Named `serialize` because `#[serde(with = "path")]` appends `::serialize`
/// to the module path it is given.
pub fn serialize<T, S, const N: usize>(array: &[T; N], serializer: S) -> Result<S::Ok, S::Error>
where
    T: Serialize,
    S: Serializer,
{
    let mut tuple = serializer.serialize_tuple(N)?;
    for element in array {
        tuple.serialize_element(element)?;
    }
    tuple.end()
}

/// Deserialize `[T; N]` from a fixed-length tuple, for any `N`.
///
/// Named `deserialize` because `#[serde(with = "path")]` appends
/// `::deserialize` to the module path it is given.
///
/// Collects into a `Vec` and converts, rather than assembling the array in
/// place through `MaybeUninit`: this is a checkpoint/restore path, not the
/// zero-copy hot path, and one bounded allocation is a better trade than
/// hand-written `unsafe` with a partial-initialization drop guard.
pub fn deserialize<'de, T, D, const N: usize>(deserializer: D) -> Result<[T; N], D::Error>
where
    T: Deserialize<'de>,
    D: Deserializer<'de>,
{
    struct ArrayVisitor<T, const N: usize>(PhantomData<T>);

    impl<'de, T, const N: usize> Visitor<'de> for ArrayVisitor<T, N>
    where
        T: Deserialize<'de>,
    {
        type Value = [T; N];

        fn expecting(&self, formatter: &mut fmt::Formatter<'_>) -> fmt::Result {
            write!(formatter, "an array of exactly {N} elements")
        }

        fn visit_seq<A>(self, mut seq: A) -> Result<Self::Value, A::Error>
        where
            A: SeqAccess<'de>,
        {
            // `size_hint` is caller-supplied and unverified, so `N` (a compile-
            // time constant) is the only safe reservation.
            let mut collected: Vec<T> = Vec::with_capacity(N);
            for index in 0..N {
                match seq.next_element()? {
                    Some(element) => collected.push(element),
                    None => return Err(de::Error::invalid_length(index, &self)),
                }
            }
            // Cannot fail: the loop pushed exactly `N` elements. Mapped rather
            // than unwrapped so the element type needs no `Debug` bound.
            collected
                .try_into()
                .map_err(|_| de::Error::invalid_length(N, &self))
        }
    }

    deserializer.deserialize_tuple(N, ArrayVisitor::<T, N>(PhantomData))
}

#[cfg(test)]
mod tests {
    use serde::{Deserialize, Serialize};

    /// The exact shape codegen emits for `geometry_msgs/PoseWithCovariance`'s
    /// `float64[36] covariance`.
    #[derive(Serialize, Deserialize, PartialEq, Debug)]
    struct Covariance {
        #[serde(with = "super")]
        matrix: [f64; 36],
    }

    /// A big array of a NON-`Copy`, non-primitive element — the shape a
    /// `Nested[40]` schema field would produce.
    #[derive(Serialize, Deserialize, PartialEq, Debug)]
    struct Names {
        #[serde(with = "super")]
        names: [String; 40],
    }

    /// The BOUNDARY this module exists to cross, asserted from the side that
    /// can be written down.
    ///
    /// `[f64; 32]` is inside serde's blanket impls, so it deserializes with no
    /// help — which is why `generator::structs::field_needs_big_array_serde`
    /// leaves such a field alone. The other side (`[f64; 36]` on its own) is
    /// not expressible as an assertion, because naming it would fail to
    /// COMPILE; it is pinned structurally instead, by `Covariance` above
    /// needing `#[serde(with = "super")]` to build at all.
    ///
    /// If a future serde gains const-generic array impls, that structural pin
    /// stops meaning anything — but the helper also stops being needed, and
    /// the emission in `generator/structs.rs` can be deleted with it.
    #[test]
    fn serde_covers_thirty_two_elements_unaided_which_is_where_this_module_starts() {
        let at_ceiling: [f64; 32] =
            serde_json::from_str(&format!("[{}]", ["4.5"; 32].join(","))).expect("deserialize");
        assert_eq!(at_ceiling, [4.5f64; 32]);
        assert_eq!(
            super::SERDE_ARRAY_IMPL_CEILING,
            32,
            "the ceiling the generator keys on must be the one probed here"
        );
    }

    #[test]
    fn a_thirty_six_element_array_round_trips_to_a_hand_built_value() {
        // Hand-built oracle: every element distinct, so a reversal, an
        // off-by-one, or a truncation all show up as a value mismatch.
        let mut matrix = [0.0f64; 36];
        for (index, slot) in matrix.iter_mut().enumerate() {
            *slot = (index as f64) * 1.5 - 7.25;
        }
        let original = Covariance { matrix };

        let json = serde_json::to_string(&original).expect("serialize");
        let recovered: Covariance = serde_json::from_str(&json).expect("deserialize");

        assert_eq!(recovered.matrix.len(), 36);
        for index in 0..36 {
            assert_eq!(
                recovered.matrix[index],
                (index as f64) * 1.5 - 7.25,
                "element {index} did not survive the round trip"
            );
        }
        assert_eq!(recovered, original);
    }

    #[test]
    fn the_array_is_encoded_as_a_bare_sequence_of_its_elements() {
        // Byte-level oracle against a hand-written expectation: a `tuple`
        // renders as a JSON array of exactly the elements, with no length
        // prefix and no wrapper object.
        let value = Covariance {
            matrix: [2.0f64; 36],
        };
        let json = serde_json::to_string(&value).expect("serialize");
        let expected = format!("{{\"matrix\":[{}]}}", ["2.0"; 36].join(","));
        assert_eq!(json, expected);
    }

    #[test]
    fn a_non_copy_element_type_round_trips_too() {
        let names: [String; 40] = core::array::from_fn(|index| format!("link_{index}"));
        let original = Names {
            names: names.clone(),
        };

        let json = serde_json::to_string(&original).expect("serialize");
        let recovered: Names = serde_json::from_str(&json).expect("deserialize");

        for index in 0..40 {
            assert_eq!(recovered.names[index], format!("link_{index}"));
        }
        assert_eq!(recovered, original);
    }

    #[test]
    fn a_short_input_is_refused_rather_than_zero_filled() {
        let short = format!("{{\"matrix\":[{}]}}", ["1.0"; 35].join(","));
        let err = serde_json::from_str::<Covariance>(&short)
            .expect_err("35 elements must not decode into a 36-element array");
        let rendered = err.to_string();
        assert!(
            rendered.contains("36"),
            "the error should name the expected length; got: {rendered}"
        );
    }

    #[test]
    fn a_long_input_is_refused_rather_than_truncated() {
        let long = format!("{{\"matrix\":[{}]}}", ["1.0"; 37].join(","));
        assert!(
            serde_json::from_str::<Covariance>(&long).is_err(),
            "37 elements must not decode into a 36-element array"
        );
    }

    #[test]
    fn an_empty_input_is_refused() {
        assert!(
            serde_json::from_str::<Covariance>("{\"matrix\":[]}").is_err(),
            "an empty array must not decode into a 36-element array"
        );
    }
}
