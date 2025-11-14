import {MavLinkPacketRegistry} from "node-mavlink";
import * as minimalACFly from "../Owl02Lib/minimalACFly";
import * as commonACFly from "../Owl02Lib/commonACFly";

// create a registry of mappings between a message id and a data class
// 注册表，用于将消息ID映射到数据类
export const REGISTRY: MavLinkPacketRegistry = {
    ...minimalACFly.REGISTRY,
    ...commonACFly.REGISTRY,
};

