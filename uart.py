from maix import uart
from maix import pinmap
from maix import app, uart, pinmap, time, sys, err

funcs = pinmap.get_pin_functions("A19")
print(funcs)

func = pinmap.get_pin_function("A19")
print(func)

pin_function = {
        "A19": "UART1_TX",
        "A18": "UART1_RX"
    }
device = "/dev/ttyS1"

for pin, func in pin_function.items():
    err.check_raise(pinmap.set_pin_function(pin, func), f"Failed set pin{pin} function to {func}")

func = pinmap.get_pin_function("A19")
print(func)

func = pinmap.get_pin_function("A18")
print(func)

# Init UART
serial_dev = uart.UART(device, 115200)
serial_dev.write_str("Hello MaixPy")