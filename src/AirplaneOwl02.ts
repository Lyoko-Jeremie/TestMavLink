import moment from "moment";
import {
    AirplaneOwl02Interface,
    AirplaneOwl02State,
} from "./AirplaneOwl02Interface";
import {AirplaneManagerOwl02Interface} from "./AirplaneManagerOwl02Interface";
import {MavLinkData, MavLinkPacket, uint8_t} from "node-mavlink";
import {MavLinkPacket2DataProcessType, PackAndDataType} from "./CustomProtocolTransformManager";
import {TimeBasedFifoCache} from "./utils/TimeBasedFifoCache";
import {BehaviorSubject, filter, firstValueFrom, Subject} from "rxjs";
import {timeoutWithoutError} from "./utils/rxjsTimeoutWithoutError";
import * as commonACFly from './Owl02Lib/commonACFly';
import * as minimalACFly from './Owl02Lib/minimalACFly';
import {getNowTimestampMsUintFloat, mathMod} from "./AirplaneTimestamp";
import {PortStateEventInterface} from "./PortStateEventInterface";

// 命令应答结果常量（根据协议文档）
const RECEIVE_COMMAND = 1;  // 接收到命令
const FINISH_COMMAND = 2;  // 完成动作
const COMMAND_ERROR = 3;  // 拒绝执行指令

/**
 * 命令状态追踪
 */
class CommandStatus {
    command: number;
    sequence: number;  // 命令序列号，用于区分同一命令的不同调用
    timestamp: number;  // Param7时间戳（整数毫秒），用于避免重复包
    receiveCount: number = 0;  // 接收应答计数
    finishCount: number = 0;  // 完成应答计数
    isReceived: boolean = false;
    isFinished: boolean = false;
    isError: boolean = false;
    isStopped: boolean = false;  // 标记是否被停止
    lastUpdate: number;  // 最后更新时间
    createTime: number;  // 创建时间

    constructor(command: number, sequence: number, timestamp: number) {
        this.command = command;
        this.sequence = sequence;
        this.timestamp = timestamp;
        this.lastUpdate = Date.now();
        this.createTime = Date.now();
    }
}

export interface MavLinkPacketRecord<D extends MavLinkData = MavLinkData> {
    time: moment.Moment;
    pack: MavLinkPacket;
    msgId: uint8_t;
    data?: MavLinkPacket2DataProcessType<D>;
}

function ListSwitch<I extends number | string, R>(o: I, s: Record<I, R>, d: R): R {
    // if (o in s) {
    //     return s[o];
    // }
    // return d;
    return s[o] ?? d;
}

export class AirplaneOwl02 implements AirplaneOwl02Interface {
    protected state: AirplaneOwl02State = new AirplaneOwl02State();
    protected cachedPacketRecord: Map<uint8_t, MavLinkPacketRecord> = new Map<uint8_t, MavLinkPacketRecord>();

    protected parseTable: { [key: number]: (data: PackAndDataType) => void };
    protected cachedPacketIds: Set<number>;

    protected cacheStateText: TimeBasedFifoCache<MavLinkPacketRecord<commonACFly.StatusText>> = new TimeBasedFifoCache({
        timeout: 1000 * 60 * 5,
        maxSize: 30,
    });

    public commander: AirplaneOwl02Commander;
    protected packStream = new Subject<MavLinkPacketRecord>();
    protected ackPackStream = new BehaviorSubject<PackAndDataType<commonACFly.CommandAck> | undefined>(undefined);

    // 命令状态追踪 - 使用 "command-sequence" 作为键
    protected commandStatus: Map<string, CommandStatus> = new Map();
    protected commandSequence: number = 0;  // 命令序列号生成器
    protected currentActiveCommandKey?: string;  // 当前正在执行的命令序列号（用于指令队列模式）

    // 重发配置
    protected maxRetries: number = 3;  // 最大重发次数
    protected retryTimeout: number = 2000;  // 重发超时时间（毫秒）
    protected asyncMode: boolean = true;  // 异步模式：不阻塞等待应答
    protected queueMode: boolean = true;  // 队列模式，新指令到来时停止旧指令重试

    protected portCloseEventSub;

    constructor(
        public targetChannelId: number,
        public manager: AirplaneManagerOwl02Interface,
        public portStateEvent: PortStateEventInterface,
    ) {
        this.parseTable = {
            [minimalACFly.Heartbeat.MSG_ID]: this.parseHeartbeat.bind(this),
            [commonACFly.ExtendedSysState.MSG_ID]: this.parseLandState.bind(this),
            [commonACFly.AutopilotVersion.MSG_ID]: this.parseAutopilotVersion.bind(this),
            [commonACFly.StatusText.MSG_ID]: this.parseStatusText.bind(this),
            [commonACFly.CommandAck.MSG_ID]: this.parseAck.bind(this),
            [commonACFly.BatteryStatusAcfly.MSG_ID]: this.parseBatteryStatusAcfly.bind(this),
        };
        this.cachedPacketIds = new Set<number>([
            // 飞控解算位置
            // 报文MAVLINK_MSG_ID_GLOBAL_POSITION_INT ，ID=33，2HZ
            commonACFly.GlobalPositionInt.MSG_ID,
            commonACFly.GpsRawInt.MSG_ID,
            commonACFly.Gps2Raw.MSG_ID,
            commonACFly.VfrHud.MSG_ID,
            commonACFly.Attitude.MSG_ID,
            commonACFly.RcChannels.MSG_ID,
            commonACFly.RcChannelsScaled.MSG_ID,
            commonACFly.MissionCurrent.MSG_ID,
            commonACFly.BatteryStatus.MSG_ID,
            commonACFly.BatteryStatusAcfly.MSG_ID,
            commonACFly.SystemTime.MSG_ID,
            commonACFly.SysStatus.MSG_ID,
        ]);
        this.commander = new AirplaneOwl02Commander(this);
        this.portCloseEventSub = this.portStateEvent.portCloseEvent.subscribe({
            next: () => {
                // closed port , stop all commands
                this.stopAllCommands();
            },
        })
    }

    protected isInit = false;

    public async init() {
        if (this.isInit) {
            return;
        }
        this.isInit = true;

        await this.sendHeartbeat();
    }

    protected cachePacketRecord(pack: MavLinkPacket, data?: MavLinkPacket2DataProcessType): void {
        const msgId = pack.header.msgid;
        const record: MavLinkPacketRecord = {
            time: moment(),
            pack: pack,
            msgId: msgId,
            data: data,
        };
        this.cachedPacketRecord.set(msgId, record);
        this.packStream.next(record);
    }

    public async sendMsg<M extends Exclude<MavLinkData, commonACFly.CommandLong>>(msg: M) {
        if (!this.portStateEvent.portIsOpen()) {
            return undefined;
        }
        return this.manager.m.sendMsg(msg, this.targetChannelId);
    }

    public async sendMsgCommand<M extends commonACFly.CommandLong>(msg: M) {
        if (!this.portStateEvent.portIsOpen()) {
            return undefined;
        }
        msg.targetComponent = 1;
        msg.targetSystem = 1;
        msg._param7 = getNowTimestampMsUintFloat();
        return this.manager.m.sendMsg(msg, this.targetChannelId);
    }

    /**
     * 发送命令并自动重试（支持异步非阻塞模式）
     * @param msg 命令消息
     * @param waitForFinish 是否等待命令完成
     * @param timeout 等待超时时间（毫秒）
     * @param asyncMode 是否异步模式（undefined时使用实例默认值）
     * @return 异步模式返回Promise对象，同步模式返回是否成功
     */
    public async sendMsgCommandWithRetry<M extends commonACFly.CommandLong>(
        msg: M,
        waitForFinish: boolean = false,
        timeout: number = 5000,
        asyncMode?: boolean
    ): Promise<boolean> {
        if (!this.portStateEvent.portIsOpen()) {
            return false;
        }

        // 确定是否使用异步模式
        const useAsync = asyncMode !== undefined ? asyncMode : this.asyncMode;

        // 生成命令序列号和时间戳
        const sequence = this.getNextSequence();
        // 使用当前时间戳（毫秒级整数），限制在 0 到 8388607 (2^23 - 1) 范围内
        const timestamp = Math.floor(Date.now()) & 0x7FFFFF;
        const key = `${msg.command}-${sequence}`;

        // 准备消息
        msg.targetComponent = 1;
        msg.targetSystem = 1;
        msg._param7 = timestamp;  // 使用时间戳作为param7

        // 创建命令状态
        const status = new CommandStatus(msg.command, sequence, timestamp);
        this.commandStatus.set(key, status);

        // 如果启用队列模式，且当前有活动命令，则先停止当前命令
        if (this.queueMode && this.currentActiveCommandKey && this.currentActiveCommandKey !== key) {
            const oldKey = this.currentActiveCommandKey;
            const oldStatus = this.commandStatus.get(oldKey);

            // 检查旧命令是否还在执行中（未收到ACK）
            if (oldStatus && !oldStatus.isReceived && !oldStatus.isFinished && !oldStatus.isStopped) {
                oldStatus.isStopped = true;
                console.warn(
                    `⚠️ 警告：新指令 ${msg.command}(seq=${sequence}, ts=${timestamp}) 到来时，` +
                    `上一个指令 ${oldStatus.command}(seq=${oldStatus.sequence}, ts=${oldStatus.timestamp}) 仍未发送成功，停止重试上一个指令`
                );
            }
        }

        // 更新当前活动命令
        this.currentActiveCommandKey = key;

        // 定义实际执行重试的函数
        const retryTask = async (): Promise<boolean> => {
            let retryCount = 0;
            const startTime = Date.now();

            while (retryCount < this.maxRetries) {
                // 检查是否已被停止
                const currentStatus = this.commandStatus.get(key);
                if (currentStatus?.isStopped) {
                    console.log(`Command ${msg.command} seq=${sequence} ts=${timestamp} stopped before sending`);
                    this.cleanupActiveCommand(key);
                    return false;
                }

                // 发送命令
                await this.manager.m.sendMsg(msg, this.targetChannelId);
                console.log(
                    `Sent command ${msg.command} seq=${sequence} ts=${timestamp} to device ${this.targetChannelId} ` +
                    `(attempt ${retryCount + 1}/${this.maxRetries})`
                );

                // 等待应答
                const waitStart = Date.now();
                while (Date.now() - waitStart < this.retryTimeout) {
                    const status = this.commandStatus.get(key);
                    if (status) {
                        if (status.isError) {
                            console.error(
                                `Command ${msg.command} seq=${sequence} ts=${timestamp} rejected by device ${this.targetChannelId}`
                            );
                            this.cleanupActiveCommand(key);
                            return false;
                        }

                        if (status.isReceived && !waitForFinish) {
                            console.log(
                                `Command ${msg.command} seq=${sequence} ts=${timestamp} received by device ${this.targetChannelId}`
                            );
                            this.cleanupActiveCommand(key);
                            return true;
                        }

                        if (status.isFinished) {
                            console.log(
                                `Command ${msg.command} seq=${sequence} ts=${timestamp} finished by device ${this.targetChannelId}`
                            );
                            this.cleanupActiveCommand(key);
                            return true;
                        }

                        // 检查是否被停止
                        if (status.isStopped) {
                            console.log(`Command ${msg.command} seq=${sequence} ts=${timestamp} stopped by new command`);
                            this.cleanupActiveCommand(key);
                            return false;
                        }
                    }

                    // 检查总超时
                    if (Date.now() - startTime > timeout) {
                        console.warn(`Command ${msg.command} seq=${sequence} ts=${timestamp} timeout after ${timeout}ms`);
                        this.cleanupActiveCommand(key);
                        return false;
                    }

                    // 等待50ms再检查
                    await new Promise(resolve => setTimeout(resolve, 50));
                }

                retryCount++;
                if (retryCount < this.maxRetries) {
                    console.warn(
                        `Command ${msg.command} seq=${sequence} ts=${timestamp} no response, retrying... ` +
                        `(${retryCount}/${this.maxRetries})`
                    );
                }
            }

            console.error(`Command ${msg.command} seq=${sequence} ts=${timestamp} failed after ${this.maxRetries} retries`);
            this.cleanupActiveCommand(key);
            return false;
        };

        if (useAsync) {
            // 异步模式：不等待，立即返回，但任务在后台执行
            retryTask().catch(err => {
                console.error(`Command ${msg.command} seq=${sequence} ts=${timestamp} async error:`, err);
            });
            return true;  // 异步模式立即返回true表示已提交
        } else {
            // 同步模式：阻塞等待完成
            return await retryTask();
        }
    }

    /**
     * 获取下一个命令序列号
     */
    protected getNextSequence(): number {
        this.commandSequence++;
        return this.commandSequence;
    }

    /**
     * 清理活动命令状态
     */
    protected cleanupActiveCommand(key: string): void {
        if (this.currentActiveCommandKey === key) {
            this.currentActiveCommandKey = undefined;
        }
    }

    /**
     * 停止所有命令
     */
    protected stopAllCommands(): void {
        for (const status of this.commandStatus.values()) {
            status.isStopped = true;
        }
        this.currentActiveCommandKey = undefined;
    }

    /**
     * 配置重试参数
     * @param maxRetries 最大重发次数，默认3次
     * @param retryTimeout 重发超时时间（毫秒），默认2000ms
     * @param asyncMode 异步模式，默认true
     * @param queueMode 队列模式（新指令到来时停止旧指令），默认true
     */
    public configureRetry(
        maxRetries?: number,
        retryTimeout?: number,
        asyncMode?: boolean,
        queueMode?: boolean
    ): void {
        if (maxRetries !== undefined) this.maxRetries = maxRetries;
        if (retryTimeout !== undefined) this.retryTimeout = retryTimeout;
        if (asyncMode !== undefined) this.asyncMode = asyncMode;
        if (queueMode !== undefined) this.queueMode = queueMode;
    }

    /**
     * 获取当前重试配置
     */
    public getRetryConfig() {
        return {
            maxRetries: this.maxRetries,
            retryTimeout: this.retryTimeout,
            asyncMode: this.asyncMode,
            queueMode: this.queueMode,
        };
    }

    public async sendHeartbeat() {
        const commandHeartbeat = new minimalACFly.Heartbeat();
        commandHeartbeat.type = minimalACFly.MavType.GCS;
        commandHeartbeat.autopilot = minimalACFly.MavAutopilot.INVALID;
        // (base_mode&0x80) ==0x80 飞机已解锁，
        // (base_mode&0x80) !=0x80 飞机未解锁.
        commandHeartbeat.baseMode = minimalACFly.MavModeFlag.CUSTOM_MODE_ENABLED;
        await this.sendMsg(commandHeartbeat);
    }

    /**
     * call this , and then the autopilot will send back AutopilotVersion pack
     */
    public async triggerGetAutopilotVersion() {
        const p = new commonACFly.RequestAutopilotCapabilitiesCommand();
        return this.sendMsgCommand(p);
    }

    protected parseHeartbeat(data: PackAndDataType) {
        const p = data.data.mavLinkData as minimalACFly.Heartbeat;
        this.state.isArmed = (p.baseMode & 0x80) === 0x80;
    }

    protected parseLandState(data: PackAndDataType) {
        const p = data.data.mavLinkData as commonACFly.ExtendedSysState;
        this.state.isLanded = p.landedState;
    }

    protected parseStatusText(data: PackAndDataType) {
        const p = data.data as MavLinkPacket2DataProcessType<commonACFly.StatusText>;
        this.cacheStateText.push({
            time: moment(),
            pack: data.packet,
            msgId: data.packet.header.msgid,
            data: p,
        });
    }

    protected parseAutopilotVersion(data: PackAndDataType) {
        const p = data.data.mavLinkData as commonACFly.AutopilotVersion;
    }

    protected parseAck(data: PackAndDataType) {
        this.ackPackStream.next(data as PackAndDataType<commonACFly.CommandAck>);
        const p = data.data.mavLinkData as commonACFly.CommandAck;

        console.log('[AirplaneOwl02] CommandAck:', p);

        const cmdId = p.command;
        const result = p.result;
        // 获取ACK中的时间戳（在resultParam2中）
        // 限制在23位范围内，与发送时保持一致
        const ackTimestamp = p.resultParam2 !== undefined ? (Math.floor(p.resultParam2) & 0x7FFFFF) : undefined;

        console.log('[AirplaneOwl02] ackTimestamp:', ackTimestamp);

        // 根据命令ID和时间戳精确匹配命令实例
        let updated = false;
        for (const [key, status] of this.commandStatus.entries()) {
            // 必须同时匹配命令ID和时间戳
            if (status.command === cmdId) {
                // 如果ACK包含时间戳，则必须匹配；否则更新所有该命令的实例（向后兼容）
                if (ackTimestamp !== undefined && status.timestamp !== ackTimestamp) {
                    continue;
                }

                status.lastUpdate = Date.now();

                if (result === RECEIVE_COMMAND) {
                    status.receiveCount++;
                    if (status.receiveCount >= 1) {
                        status.isReceived = true;
                    }
                    console.log(
                        `Command ${cmdId} seq=${status.sequence} ts=${status.timestamp} received ACK ` +
                        `(${status.receiveCount}/3)`
                    );
                    updated = true;

                } else if (result === FINISH_COMMAND) {
                    status.finishCount++;
                    if (status.finishCount >= 1) {
                        status.isFinished = true;
                    }
                    console.log(
                        `Command ${cmdId} seq=${status.sequence} ts=${status.timestamp} finished ACK ` +
                        `(${status.finishCount}/3)`
                    );
                    updated = true;

                } else if (result === COMMAND_ERROR) {
                    status.isError = true;
                    console.error(
                        `Command ${cmdId} seq=${status.sequence} ts=${status.timestamp} error from device ${this.targetChannelId}`
                    );
                    updated = true;
                }
            }
        }

        // 清理超过10秒的旧命令状态
        const currentTime = Date.now();
        const keysToRemove: string[] = [];
        for (const [key, status] of this.commandStatus.entries()) {
            if (currentTime - status.createTime > 10000) {
                keysToRemove.push(key);
            }
        }
        for (const key of keysToRemove) {
            this.commandStatus.delete(key);
            console.log(`Cleaned up old command status: ${key}`);
        }
    }

    protected parseBatteryStatusAcfly(data: PackAndDataType) {
        const p = data.data.mavLinkData as commonACFly.BatteryStatusAcfly;
        // 电池电压，单位100mv
        // 无效值: 0xFFFFFFFF
        p.voltage;
        // 电池容量，单位mAh
        // 无效值: 0xFFFFFFFF
        p.capacity;
        // 电池序列号
        // 无效值: 0xFFFFFFFF
        p.sequenceNum;
        // 错误标记位
        // 无效值0xFF
        p.faultBitmask;
        // 温度，单位度
        // 无效值0x7FFF
        p.temperature;
        // 循环次数
        // 无效值0xFFFF
        p.cycleCount;
        // 电量，单位100mA
        p.current;
        // 电池id
        // 根据此字段区分多个电池
        p.id;
        // 健康度，[0 - 100]
        p.health;
        // 剩余百分比 [0 - 100]
        p.remainingPercentage;
        // 电池节数，无效值0xFF
        // 注:航模电池一般由多节电池组成
        p.cellCount;
        // 单节电池电压，一般满电为4.2V；单位0.02mV，无效值0x00
        // 例voltages[0]=210，那么第一节电池电压为210*0.02=4.2V
        p.voltages;
    }

    public async parseStateFromMavLink(data: PackAndDataType) {
        this.cachePacketRecord(data.packet, data.data);
        const id = data.packet.header.msgid;
        const func = this.parseTable[id];
        if (func) {
            func(data);
        } else {
            if (!this.cachedPacketIds.has(id)) {
                // TODO only debug
                console.warn('[AirplaneOwl02] Unknow pack: ', id, data);
            }
        }
    }

    public getGpsPos() {
        const p = this.cachedPacketRecord.get(commonACFly.GlobalPositionInt.MSG_ID) as undefined | commonACFly.GlobalPositionInt;
        if (!p) {
            return undefined;
        }
        return {
            lat: p.lat / 1e7,
            lon: p.lon / 1e7,
            alt: p.alt / 1e3,
            relativeAlt: p.relativeAlt / 1e3,
            vx: p.vx,
            vy: p.vy,
            vz: p.vz,
            hdg: p.hdg,
        }
    }

    public getAttitude() {
        const p = this.cachedPacketRecord.get(commonACFly.Attitude.MSG_ID) as undefined | commonACFly.Attitude;
        if (!p) {
            return undefined;
        }
        return {
            roll: p.roll,
            pitch: p.pitch,
            yaw: p.yaw,
            rollSpeed: p.rollspeed,
            pitchSpeed: p.pitchspeed,
            yawSpeed: p.yawspeed,
            timeBootMs: p.timeBootMs,
        };
    }

    public getStatusTextList(): { text: string, time: moment.Moment, severity: number }[] {
        return this.cacheStateText.toArray().map(T => {
            return {
                text: T.data?.mavLinkData.text ?? '',
                time: T.time,
                severity: T.data?.mavLinkData.severity ?? 0,
            };
        });
    }

    public waitAckOnce(timeoutMs: number, command: commonACFly.MavCmd) {
        return firstValueFrom(this.ackPackStream.pipe(
            filter((T): T is PackAndDataType<commonACFly.CommandAck> => !!T),
            filter(P => P.data.mavLinkData.command === command),
            timeoutWithoutError(timeoutMs, undefined),
        ));
    }

    public destroy() {
        this.stopAllCommands();
        this.commandStatus.clear();
        this.packStream.complete();
        this.ackPackStream.complete();
        this.cacheStateText.clear();
        this.cachedPacketRecord.clear();
        this.portCloseEventSub.unsubscribe();
    }

}

export class AirplaneOwl02Commander {

    constructor(
        public airplane: AirplaneOwl02,
    ) {
    }

    /**
     * !!! invalid !!!  use land() instead
     */
    lock() {
        const p = new commonACFly.ComponentArmDisarmCommand();
        p.arm = 0;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    unlock() {
        const p = new commonACFly.ComponentArmDisarmCommand();
        p.arm = 1;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * @param height cm
     */
    takeoff(height: number) {
        const p = new commonACFly.ExtDroneTakeoffCommand();
        p.height = height;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * 一键降落
     */
    land() {
        const p = new commonACFly.ExtDroneLandCommand();
        p.land_mode = 1;
        p.landspeed = 100;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * 室内定点飞行
     * @param x       （x，cm）
     * @param y       （y，cm）
     * @param h       （高度，cm）
     */
    goto(x: number, y: number, h: number) {
        const p = new commonACFly.ExtDroneGotoCmdCommand();
        p.target_x = x;
        p.target_y = y;
        p.target_z = h;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * @param forward 1：上升 2：下降，3：前，4：后，5：左，6：右
     * @param distance
     * @param speed
     */
    move(forward: 1 | 2 | 3 | 4 | 5 | 6, distance: number, speed: number = 100) {
        const p = new commonACFly.ExtDroneMoveCommand();
        p.direction = forward;
        p.distance = distance;
        p.speed = speed;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    up(distance: number, speed?: number) {
        return this.move(1, distance, speed);
    }

    down(distance: number, speed?: number) {
        return this.move(2, distance, speed);
    }

    forward(distance: number, speed?: number) {
        return this.move(3, distance, speed);
    }

    back(distance: number, speed?: number) {
        return this.move(4, distance, speed);
    }

    left(distance: number, speed?: number) {
        return this.move(5, distance, speed);
    }

    right(distance: number, speed?: number) {
        return this.move(6, distance, speed);
    }

    /**
     * @param forward 1：逆时针 2：顺时针
     * @param degrees  转动角度（单位 度，min值0，max值360）
     */
    rotate(forward: 1 | 2, degrees: number) {
        // mod(degrees,360)
        degrees = mathMod(degrees, 360);
        const p = new commonACFly.ExtDroneCircleCommand();
        p.direction = forward;
        p.degrees = degrees;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    cw(degrees: number) {
        return this.rotate(2, degrees);
    }

    ccw(degrees: number) {
        return this.rotate(1, degrees);
    }

    setSpeed(speed: number) {
        const p = new commonACFly.ExtDroneChangeSpeedCommand();
        p.speed = speed;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    light(r: number, g: number, b: number) {
        const p = new commonACFly.ExtDroneLightRgbCommand();
        p.r = r;
        p.g = g;
        p.b = b;
        p.breathe = 0;
        p.rainbow = 0;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    rainbow(r: number, g: number, b: number) {
        const p = new commonACFly.ExtDroneLightRgbCommand();
        p.r = r;
        p.g = g;
        p.b = b;
        p.breathe = 0;
        p.rainbow = 1;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    breathe(r: number, g: number, b: number) {
        const p = new commonACFly.ExtDroneLightRgbCommand();
        p.r = r;
        p.g = g;
        p.b = b;
        p.breathe = 1;
        p.rainbow = 0;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * Return to launch - RTL command
     */
    returnToLaunch() {
        const p = new commonACFly.NavReturnToLaunchCommand();
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * Set altitude - simplified version of goto
     * @param high altitude in cm
     */
    high(high: number) {
        const p = new commonACFly.ExtDroneSetHeghtCommand();
        p.height = high;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * Set flight mode
     * @param mode 1:normal 2:track line 3:follow
     */
    setMode(mode: 1 | 2 | 3) {
        const p = new commonACFly.ExtDroneSetModeCommand();
        p.mode = mode;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * Set color detection mode
     * @param l_min L channel min value
     * @param l_max L channel max value
     * @param a_min A channel min value
     * @param a_max A channel max value
     * @param b_min B channel min value
     * @param b_max B channel max value
     */
    setColorDetectMode(l_min: number, l_max: number, a_min: number, a_max: number, b_min: number, b_max: number) {
        const p = new commonACFly.ExtDroneVisionDetectModeSetCommand();
        p.l_l = l_min;
        p.l_h = l_max;
        p.a_l = a_min;
        p.a_h = a_max;
        p.b_l = b_min;
        p.b_h = b_max;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * Emergency stop - stop motors immediately
     */
    emergency_stop() {
        const p = new commonACFly.ExtDroneUrgentDisarmCommand();
        p.cmd = 1; // disarm
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * Hover in place - stop current movement
     */
    hover() {
        const p = new commonACFly.ExtDroneHoverCommand();
        p.cmd = 1;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * Flip forward
     */
    flipForward() {
        const p = new commonACFly.ExtDroneExtraActionsCommand();
        p.roll = 1;
        p._param2 = 1; // forward
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * Flip backward
     */
    flipBack() {
        const p = new commonACFly.ExtDroneExtraActionsCommand();
        p.roll = 1;
        p._param2 = 2; // backward
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * Flip left
     */
    flipLeft() {
        const p = new commonACFly.ExtDroneExtraActionsCommand();
        p.roll = 1;
        p._param2 = 3; // left
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * Flip right
     */
    flipRight() {
        const p = new commonACFly.ExtDroneExtraActionsCommand();
        p.roll = 1;
        p._param2 = 4; // right
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * 设置openmv识别模式
     * @param mode (1常规 2巡线 3跟随)
     */
    set_openmv_mode(mode: number) {
        const p = new commonACFly.ExtDroneSetModeCommand();
        p.mode = mode;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

    /**
     * 开始OPENMV相关运动 MAV_CMD_EXT_DRONE_OPEMMV_CMD
     * 开启之前要通过 MAV_CMD_EXT_DRONE_SET_MODE / MAV_CMD_EXT_DRONE_VISION_DETECT_MODE_SET 对应的识别模式和设置
     * @param cmd  视觉模式值 (0:巡线 1:锁定二维码，飞到二维码正上方，3：寻找色块)
     * @param x    x轴移动距离
     * @param y    y轴移动距离
     * @param z    z轴移动距离
     */
    go_openmv_cmd(cmd: number, x: number, y: number, z: number) {
        const p = new commonACFly.ExtDroneOpemmvCmdCommand();
        p.cmd = cmd;
        p.xDistance = x;
        p.yDistance = y;
        p.zDistance = z;
        return this.airplane.sendMsgCommandWithRetry(p);
    }

}
