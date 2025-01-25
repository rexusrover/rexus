import typing as T
import logging
from dataclasses import dataclass, field

from eureka.config.device_config import (
    DeviceConfigCommand,
    ClientDefinitionDict,
    SimulationState,
    create_default_simulation_field,
)
from eureka.config.device_config.types import (
    IPAddress,
    StrLiteral,
    StrXMLURDF,
    URDF_XML_FIELD,
)
from eureka.config.device_config.fields_utility import (
    create_default_simulation_field,
    create_default_socket_timeout_field,
    create_default_speed_level_field
)

from eureka.config.device_config.robots._base import _URDFBasedRobotConfig
from eureka.config.device_config.robots.robots import SimRosConfig, _RobotSimulationConfig
logger = logging.getLogger(__name__)

@dataclass
class RexusRobotConfig(_RobotSimulationConfig, _URDFBasedRobotConfig):
    """Configuration for Rexus Robots
    """

    device_type: T.ClassVar[str] = "RexusRobot"
    name: str = "rexus_robot"
    model: str = field(
        default="rexus_arm_v3",
        metadata={
            "tooltip": "Model of the robot.",
            },
    )
    device_path: str = field(
        default_factory=str,
        metadata={
            "name": "Device Path",
            "tooltip": "Path the serial device under /dev/",
        },
    )
    baudrate: int = field(
        default=19200,
        metadata={
            "name": "Baudrate",
        },
    )
    timeout: float = field(
        default=3,
        metadata={
            "name": "Timeout",
        },
    )    
    
    trajectory_blending_radius: int = field(
        default=30,
        metadata={
            "tooltip": "Used with the RobotMove node to smoothen the trajectory thorugh multiple points",
            "min": 1,
        },
    )

    io_config: T.Dict[str, str] = field(
        default_factory=dict,
        metadata={
            "tooltip": """Configuration of the ports.
                Use the teach pendant to add IO ports to be controller in the format <IO Name>:<IO Type("din"/"dout")>.
                eg.
                {
                    "IIO_di1":"din",
                    "IIO_di1":"dout"
                }
            """
        },
    )

    @property
    def port_config(self):
        """IO port configuration.

        Used by nodes to calculate the ports names.
        """
        ports = {}
        for idx, (k, v) in enumerate(self.io_config.items()):
            ports[k] = [idx, v, False]
        return {
            "controller": {
                "ipaddress": self.ip,
                "unitno": 0,
                "port": 0,
                "portstat": ports,
            }
        }

    urdf_extra: StrXMLURDF = field(**URDF_XML_FIELD)
    speed_level: float = create_default_speed_level_field()
    simulation: SimulationState = create_default_simulation_field()

    @property
    def urdf_filename(self):
        return f"package://rexus_description/urdf/{self.model}.urdf"

    @property
    def srdf_filename(self):
        return f"package://rexus_description/srdf/{self.model}.srdf"

    @property
    def client_definition(self) -> ClientDefinitionDict:
        if self.simulation:
            cdef: ClientDefinitionDict = {
                    "module_name": "eureka.drivers.robot.simu",
                    "class_name": "RobotSimuClient",
                    "args": [],
                    "kwargs": {"ros_namespace": self.sim_config.ros_namespace},
                }
        else:
            cdef = {
                "module_name": "rexus.rexus_client",
                "class_name": "RobotController",
                "args": [],
                "kwargs": {
                    "namespace": self.sim_config.ros_namespace,
                    "trajectory_blending_radius": self.trajectory_blending_radius,
                },
            }
        return cdef

    @property
    def commands(self) -> T.List[DeviceConfigCommand]:
        return []