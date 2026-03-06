package networks;

import org.car2x.veins.base.connectionManager.ConnectionManager;
import org.car2x.veins.base.modules.BaseWorldUtility;
import org.car2x.veins.modules.mobility.traci.TraCIScenarioManagerForker;
import org.car2x.veins.modules.world.annotations.AnnotationManager;
import org.car2x.veins.modules.obstacle.ObstacleControl;
import org.car2x.veins.nodes.Car;

// Minimal but complete Veins network: SUMO vehicles are instantiated dynamically by the manager.
network TwoCarsScenario
{
    parameters:
        @display("bgb=900,600");
    submodules:
        connectionManager: ConnectionManager { @display("p=80,60;i=block/wifil"); }
        world: BaseWorldUtility { @display("p=80,130;i=block/cogwheel"); }
        obstacles: ObstacleControl { @display("p=80,200;i=block/brick"); }
        annotations: AnnotationManager { @display("p=80,270;i=block/pencil"); }
        manager: TraCIScenarioManagerForker { @display("p=80,340;i=block/network"); }
        node[0]: Car {}
    connections allowunconnected:
}
