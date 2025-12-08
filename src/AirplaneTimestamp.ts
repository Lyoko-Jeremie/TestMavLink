export function mathMod(a: number, b: number): number {
    return ((a % b) + b) % b;
}

export function getNowTimestampMsUintFloat(): number {
    // 使用当前时间戳（毫秒级整数），如果param7已指定则使用指定值
    // 限制在 0 到 8388607 (2^23 - 1) 范围内，确保float能精确表示
    const t = Math.floor(Date.now());
    // mod 8388608 to fit in float precision
    return t  & 0x7FFFFF;  // 限制在23位
}
