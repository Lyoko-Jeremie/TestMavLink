import {common} from "node-mavlink";

export interface AirplaneOwl02Interface {
    targetChannelId: number;
}

// TODO flyMode
// // custom_mode的第3个字节代表飞机的主模式，定义如下
// // 定高模式（2）
// // 定点模式（3）
// // 自动模式（4）
// export enum FlyModeEnum {
//     FLY_MODE_HOLD = 2,
//     FLY_MODE_POSITION = 3,
//     FLY_MODE_AUTO = 4,
//     FLY_MODE_OFF_BOARD = 4,
//     INVALID = 16,
// }
//
// // custom_mode的第4个字节代表飞机的细分模式，定义如下
// // 自动模式细分：
// // 	自动起飞模式（2）
// // 	自动跟踪模式（3）
// // 	自动任务模式（4）
// // 	自动返航模式（5）
// // 	自动降落模式（6）
// // 定点模式细分：
// // 	普通定点模式（0）
// // 	定点避障模式（2）
// export enum FlyModeAutoEnum {
//     FLY_MODE_AUTO_TAKEOFF = 2,
//     FLY_MODE_AUTO_FOLLOW = 3,
//     FLY_MODE_AUTO_MISSION = 4,
//     FLY_MODE_AUTO_RTL = 5,
//     FLY_MODE_AUTO_LAND = 6,
//     INVALID = 16,
// }

export enum FlyModeStableEnum {
    FLY_MODE_STABLE_NORMAL = 0,
    FLY_MODE_STABLE_OBSTACLE_AVOIDANCE = 2,
    INVALID = 16,
}

export class AirplaneOwl02State {
    isArmed: boolean = false;
    // TODO flyMode
    // flyMode: FlyModeEnum = FlyModeEnum.INVALID;
    // flyModeAuto: FlyModeAutoEnum = FlyModeAutoEnum.INVALID;
    flyModeStable: FlyModeStableEnum = FlyModeStableEnum.INVALID;

    isLanded: common.MavLandedState = common.MavLandedState.UNDEFINED;
    flightSwVersion?: number;
    flightSwVersionString?: string;
    boardVersion?: number;
    SN?: string;

    gpsPosition: {
        lat: number,
        lon: number,
        alt: number,
        relativeAlt: number,
        hdg: number,
    } = {
        lat: 0,
        lon: 0,
        alt: 0,
        relativeAlt: 0,
        hdg: 0,
    };
}
