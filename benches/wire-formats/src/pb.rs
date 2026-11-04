//! prost structs for schema/wire.proto, written as prost-build emits them.
//! `*V` types hold `bytes` as `Vec<u8>` (prost's default); `*B` types hold
//! `bytes::Bytes` (prost-build's `.bytes(["."])` option), which decodes the
//! payload as a zero-copy slice of a `Bytes` input buffer.
use prost::Message;

#[derive(Clone, PartialEq, Message)]
pub struct Time {
    #[prost(int32, tag = "1")]
    pub sec: i32,
    #[prost(uint32, tag = "2")]
    pub nanosec: u32,
}
#[derive(Clone, PartialEq, Message)]
pub struct Header {
    #[prost(message, optional, tag = "1")]
    pub stamp: Option<Time>,
    #[prost(string, tag = "2")]
    pub frame_id: String,
}
#[derive(Clone, PartialEq, Message)]
pub struct ImuSample {
    #[prost(uint64, tag = "1")]
    pub stamp_ns: u64,
    #[prost(uint32, tag = "2")]
    pub seq: u32,
    #[prost(uint32, tag = "3")]
    pub status: u32,
    #[prost(double, tag = "4")]
    pub ox: f64,
    #[prost(double, tag = "5")]
    pub oy: f64,
    #[prost(double, tag = "6")]
    pub oz: f64,
    #[prost(double, tag = "7")]
    pub ow: f64,
    #[prost(double, tag = "8")]
    pub avx: f64,
    #[prost(double, tag = "9")]
    pub avy: f64,
    #[prost(double, tag = "10")]
    pub avz: f64,
    #[prost(double, tag = "11")]
    pub lax: f64,
    #[prost(double, tag = "12")]
    pub lay: f64,
    #[prost(double, tag = "13")]
    pub laz: f64,
}
#[derive(Clone, PartialEq, Message)]
pub struct ImageV {
    #[prost(message, optional, tag = "1")]
    pub header: Option<Header>,
    #[prost(uint32, tag = "2")]
    pub height: u32,
    #[prost(uint32, tag = "3")]
    pub width: u32,
    #[prost(string, tag = "4")]
    pub encoding: String,
    #[prost(uint32, tag = "5")]
    pub is_bigendian: u32,
    #[prost(uint32, tag = "6")]
    pub step: u32,
    #[prost(bytes = "vec", tag = "7")]
    pub data: Vec<u8>,
}
#[derive(Clone, PartialEq, Message)]
pub struct ImageB {
    #[prost(message, optional, tag = "1")]
    pub header: Option<Header>,
    #[prost(uint32, tag = "2")]
    pub height: u32,
    #[prost(uint32, tag = "3")]
    pub width: u32,
    #[prost(string, tag = "4")]
    pub encoding: String,
    #[prost(uint32, tag = "5")]
    pub is_bigendian: u32,
    #[prost(uint32, tag = "6")]
    pub step: u32,
    #[prost(bytes = "bytes", tag = "7")]
    pub data: bytes::Bytes,
}
#[derive(Clone, PartialEq, Message)]
pub struct PointField {
    #[prost(string, tag = "1")]
    pub name: String,
    #[prost(uint32, tag = "2")]
    pub offset: u32,
    #[prost(uint32, tag = "3")]
    pub datatype: u32,
    #[prost(uint32, tag = "4")]
    pub count: u32,
}
#[derive(Clone, PartialEq, Message)]
pub struct PointCloud2V {
    #[prost(message, optional, tag = "1")]
    pub header: Option<Header>,
    #[prost(uint32, tag = "2")]
    pub height: u32,
    #[prost(uint32, tag = "3")]
    pub width: u32,
    #[prost(message, repeated, tag = "4")]
    pub fields: Vec<PointField>,
    #[prost(bool, tag = "5")]
    pub is_bigendian: bool,
    #[prost(uint32, tag = "6")]
    pub point_step: u32,
    #[prost(uint32, tag = "7")]
    pub row_step: u32,
    #[prost(bytes = "vec", tag = "8")]
    pub data: Vec<u8>,
    #[prost(bool, tag = "9")]
    pub is_dense: bool,
}
#[derive(Clone, PartialEq, Message)]
pub struct PointCloud2B {
    #[prost(message, optional, tag = "1")]
    pub header: Option<Header>,
    #[prost(uint32, tag = "2")]
    pub height: u32,
    #[prost(uint32, tag = "3")]
    pub width: u32,
    #[prost(message, repeated, tag = "4")]
    pub fields: Vec<PointField>,
    #[prost(bool, tag = "5")]
    pub is_bigendian: bool,
    #[prost(uint32, tag = "6")]
    pub point_step: u32,
    #[prost(uint32, tag = "7")]
    pub row_step: u32,
    #[prost(bytes = "bytes", tag = "8")]
    pub data: bytes::Bytes,
    #[prost(bool, tag = "9")]
    pub is_dense: bool,
}
