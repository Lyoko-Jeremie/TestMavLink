import {CustomProtocolTransformManager} from "./CustomProtocolTransformManager";
import {AirplaneManagerOwl02Interface} from "./AirplaneManagerOwl02Interface";
import {AirplaneOwl02} from "./AirplaneOwl02";
import {Subscription} from "rxjs";
import {UtilTimer} from "./utils/UtilTimer";

export class AirplaneManagerOwl02 implements AirplaneManagerOwl02Interface {
    protected airplane: Map<number, AirplaneOwl02> = new Map<number, AirplaneOwl02>();
    protected uSubscription?: Subscription;
    protected timerHeartbeat: UtilTimer;

    constructor(
        public m: CustomProtocolTransformManager,
    ) {
        this.timerHeartbeat = new UtilTimer(
            async () => {
                for (const [id, a] of this.airplane) {
                    await a.sendHeartbeat();
                }
            },
            console,
            1000 * 2
        )
    }

    protected isInit = false;

    public init() {
        if (this.isInit) {
            return;
        }
        this.isInit = true;

        this.uSubscription = this.m.getMavLinkAllPackAndDataObservable().subscribe({
            next: (data) => {
                // 处理接收到的数据
                console.log(`====== Received data [${data.id}]:`, data);
                this.getAirplane(data.id)
                    .then((a) => {
                        return a.parseStateFromMavLink(data.packAndData);
                    }).catch(e => console.error('AirplaneManagerOwl02 DataObservable Error : ', e));
            },
        });
    }

    public async getAirplane(id: number) {
        let airplane = this.airplane.get(id);
        if (!airplane) {
            airplane = new AirplaneOwl02(id, this);
            await airplane.init();
            this.airplane.set(id, airplane);
        }
        return airplane;
    }

    public destroy() {
        try {
            this.uSubscription?.unsubscribe();
        } catch (e) {
            console.error(e);
            // simple ignore it
        }
        try {
            this.timerHeartbeat.stop();
        } catch (e) {
            console.error(e);
            // simple ignore it
        }
        this.airplane.forEach(a => a.destroy());
        this.airplane.clear();
    }
}
