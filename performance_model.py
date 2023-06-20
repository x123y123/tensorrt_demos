import time
from typing import Any, Callable, List, TypeVar

F = TypeVar("F", bound=Callable[..., Any])

def performance_monitor(func: F) -> F:
    def wrapper(*args: Any, **kwargs: Any) -> Any:
        start_time = time.time()
        result = func(*args, **kwargs)
        end_time = time.time()
        execution_time = end_time - start_time
        print(f"{func.__name__} took {execution_time} seconds to execute.") 
        return result
    wrapper.execution_times: List[float] = []
    return wrapper

def set_freq(data: int) -> None:
    file = open("/sys/devices/system/cpu/cpu0/cpufreq/scaling_setspeed", "w")
    file.write(str(data))
    file.close()
def read_power() -> int:
    curr = open("/sys/bus/i2c/drivers/ina3221/7-0040/hwmon/hwmon5/curr2_input", "r")
    volt = open("/sys/bus/i2c/drivers/ina3221/7-0040/hwmon/hwmon5/in2_input", "r")
    curr_data = int(curr.read())
    volt_data = int(volt.read())
    curr.close()
    volt.close()

    return curr_data * volt_data / 1000000

