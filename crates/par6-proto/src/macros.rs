//! Crate-internal macros.

/// Defines a wire enum plus the reflection needed by codegen and decode:
/// `ALL`, `variants()` (name/value pairs) and `from_wire()`.
///
/// Keeping enum definition and reflection in one macro is what guarantees the
/// generated Python mirror can never drift from the Rust enums.
macro_rules! wire_enum {
    (
        $(#[$meta:meta])*
        $name:ident: $repr:ty {
            $($(#[$vmeta:meta])* $variant:ident = $value:literal,)+
        }
    ) => {
        $(#[$meta])*
        #[derive(Debug, Clone, Copy, PartialEq, Eq, Hash)]
        #[repr($repr)]
        pub enum $name {
            $($(#[$vmeta])* $variant = $value,)+
        }

        impl $name {
            /// Every variant, in declaration order.
            pub const ALL: &'static [$name] = &[$($name::$variant,)+];

            /// `(rust_name, wire_value)` pairs, in declaration order.
            pub fn variants() -> &'static [(&'static str, i64)] {
                &[$((stringify!($variant), $value),)+]
            }

            /// Map a wire integer back to the enum, if it names a variant.
            pub fn from_wire(v: i64) -> Option<Self> {
                $(if v == $value {
                    return Some($name::$variant);
                })+
                None
            }
        }

        // Deserialized from the WIRE INTEGER, not the variant name: a
        // caller that already speaks the wire (the Python binding sends
        // the same numbers it puts on the socket) should not have to
        // spell the names a second way.
        impl<'de> serde::Deserialize<'de> for $name {
            fn deserialize<D: serde::Deserializer<'de>>(d: D) -> Result<Self, D::Error> {
                let v = i64::deserialize(d)?;
                $name::from_wire(v).ok_or_else(|| {
                    serde::de::Error::custom(format!(
                        "{} is not a {}",
                        v,
                        stringify!($name)
                    ))
                })
            }
        }
    };
}
