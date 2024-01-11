#[derive(Debug)]
#[repr(C)]
struct MetaData {
    magic: u64,
    crc: u32,
    meta_version: u32,
    storage_addr: usize,
    hw_version: [u8; 4],
    sw_version: [u8; 4],
    copy_func: usize,
    new_app: usize,
    new_app_len: usize,
    new_app_dest: usize,
}

magic: 0x1c80_73ab_2085_3579
