/**
 * 基于时间的先进先出缓存
 * 支持顺序访问、最大/最小数量限制、push/pop操作
 */
export interface CacheItem<T> {
    /** 数据项 */
    data: T;
    /** 时间戳 */
    timestamp: number;
}

export interface TimeBasedFifoCacheOptions {
    /** 最大容量，默认100 */
    maxSize?: number;
    /** 最小容量，默认0 */
    minSize?: number;
    /** 超时时间(毫秒)，0表示不超时，默认0 */
    timeout?: number;
}

export class TimeBasedFifoCache<T> {
    private items: CacheItem<T>[] = [];
    private readonly maxSize: number;
    private readonly minSize: number;
    private readonly timeout: number;

    constructor(options: TimeBasedFifoCacheOptions = {}) {
        this.maxSize = options.maxSize ?? 100;
        this.minSize = options.minSize ?? 0;
        this.timeout = options.timeout ?? 0;

        if (this.maxSize < this.minSize) {
            throw new Error('maxSize不能小于minSize');
        }
        if (this.minSize < 0) {
            throw new Error('minSize不能小于0');
        }
    }

    /**
     * 添加新项到缓存末尾
     * @param data 要添加的数据
     */
    push(data: T): void {
        const now = Date.now();
        this.items.push({
            data,
            timestamp: now
        });

        // 清理过期项
        this.cleanExpired();

        // 如果超过最大容量，移除最旧的项
        while (this.items.length > this.maxSize) {
            this.items.shift();
        }
    }

    /**
     * 从缓存头部取出最旧的项
     * @returns 最旧的项，如果缓存为空则返回undefined
     */
    pop(): T | undefined {
        this.cleanExpired();

        if (this.items.length <= this.minSize) {
            return undefined;
        }

        const item = this.items.shift();
        return item?.data;
    }

    /**
     * 从缓存尾部取出最新的项
     * @returns 最新的项，如果缓存为空则返回undefined
     */
    popLast(): T | undefined {
        this.cleanExpired();

        if (this.items.length <= this.minSize) {
            return undefined;
        }

        const item = this.items.pop();
        return item?.data;
    }

    /**
     * 查看头部项但不移除
     * @returns 最旧的项，如果缓存为空则返回undefined
     */
    peek(): T | undefined {
        this.cleanExpired();
        return this.items[0]?.data;
    }

    /**
     * 查看尾部项但不移除
     * @returns 最新的项，如果缓存为空则返回undefined
     */
    peekLast(): T | undefined {
        this.cleanExpired();
        return this.items[this.items.length - 1]?.data;
    }

    /**
     * 获取指定索引的项
     * @param index 索引，0为最旧的项
     * @returns 指定索引的项，如果索引无效则返回undefined
     */
    get(index: number): T | undefined {
        this.cleanExpired();

        if (index < 0 || index >= this.items.length) {
            return undefined;
        }

        return this.items[index]?.data;
    }

    /**
     * 获取所有项的数据数组（按时间顺序，最旧的在前）
     * @returns 数据数组
     */
    toArray(): T[] {
        this.cleanExpired();
        return this.items.map(item => item.data);
    }

    /**
     * 获取所有项的详细信息数组
     * @returns 包含数据和时间戳的数组
     */
    toDetailArray(): CacheItem<T>[] {
        this.cleanExpired();
        return [...this.items];
    }

    /**
     * 遍历缓存中的所有项
     * @param callback 回调函数
     */
    forEach(callback: (data: T, index: number, timestamp: number) => void): void {
        this.cleanExpired();
        this.items.forEach((item, index) => {
            callback(item.data, index, item.timestamp);
        });
    }

    /**
     * 查找满足条件的第一个项
     * @param predicate 查找条件
     * @returns 找到的项，如果没找到则返回undefined
     */
    find(predicate: (data: T, index: number, timestamp: number) => boolean): T | undefined {
        this.cleanExpired();

        for (let i = 0; i < this.items.length; i++) {
            const item = this.items[i];
            if (predicate(item.data, i, item.timestamp)) {
                return item.data;
            }
        }

        return undefined;
    }

    /**
     * 过滤缓存中的项
     * @param predicate 过滤条件
     * @returns 满足条件的项数组
     */
    filter(predicate: (data: T, index: number, timestamp: number) => boolean): T[] {
        this.cleanExpired();

        return this.items
            .filter((item, index) => predicate(item.data, index, item.timestamp))
            .map(item => item.data);
    }

    /**
     * 获取缓存大小
     * @returns 当前缓存中的项数
     */
    get size(): number {
        this.cleanExpired();
        return this.items.length;
    }

    /**
     * 检查缓存是否为空
     * @returns 是否为空
     */
    get isEmpty(): boolean {
        this.cleanExpired();
        return this.items.length === 0;
    }

    /**
     * 检查缓存是否已满
     * @returns 是否已满
     */
    get isFull(): boolean {
        this.cleanExpired();
        return this.items.length >= this.maxSize;
    }

    /**
     * 清空缓存
     */
    clear(): void {
        this.items = [];
    }

    /**
     * 获取最旧项的时间戳
     * @returns 最旧项的时间戳，如果缓存为空则返回undefined
     */
    get oldestTimestamp(): number | undefined {
        this.cleanExpired();
        return this.items[0]?.timestamp;
    }

    /**
     * 获取最新项的时间戳
     * @returns 最新项的时间戳，如果缓存为空则返回undefined
     */
    get newestTimestamp(): number | undefined {
        this.cleanExpired();
        return this.items[this.items.length - 1]?.timestamp;
    }

    /**
     * 获取缓存时间跨度（毫秒）
     * @returns 时间跨度，如果缓存为空则返回0
     */
    get timeSpan(): number {
        this.cleanExpired();

        if (this.items.length === 0) {
            return 0;
        }

        if (this.items.length === 1) {
            return 0;
        }

        return this.newestTimestamp! - this.oldestTimestamp!;
    }

    /**
     * 清理过期项
     */
    private cleanExpired(): void {
        if (this.timeout <= 0) {
            return;
        }

        const now = Date.now();
        const cutoffTime = now - this.timeout;

        while (this.items.length > this.minSize &&
               this.items[0] &&
               this.items[0].timestamp < cutoffTime) {
            this.items.shift();
        }
    }

    /**
     * 获取缓存配置信息
     */
    get config(): Required<TimeBasedFifoCacheOptions> {
        return {
            maxSize: this.maxSize,
            minSize: this.minSize,
            timeout: this.timeout
        };
    }
}
