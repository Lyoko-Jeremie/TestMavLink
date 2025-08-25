import {CustomProtocolTransformManager} from "./CustomProtocolTransformManager";
import {AirplaneManagerOwl02Interface} from "./AirplaneManagerOwl02Interface";
import {AirplaneOwl02} from "./AirplaneOwl02";
import {Subscription} from "rxjs";

export class AirplaneManagerOwl02 implements AirplaneManagerOwl02Interface {
    airplane: Map<number, AirplaneOwl02> = new Map<number, AirplaneOwl02>();
    uSubscription?: Subscription;

    constructor(
        public m: CustomProtocolTransformManager,
    ) {
    }

    init() {
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

    protected async getAirplane(id: number) {
        let airplane = this.airplane.get(id);
        if (!airplane) {
            airplane = new AirplaneOwl02(id, this);
            await airplane.init();
            this.airplane.set(id, airplane);
        }
        return airplane;
    }
}
