from .sensorbox.utils import parse_args
from .sensorbox.sensorbox import SensorBox

opts = parse_args()
print(opts)
box = SensorBox(opts)
box.run()

