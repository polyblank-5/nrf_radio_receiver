# Build Dependencies

```bash
# Build dependencies
sudo apt install python3-pip cmake curl ninja-build
pip install west

# installing Zephyr SDK for ARM only in ~/.local. See Zephyr documentation for other possible location.
mkdir -p $HOME/.local
curl -L https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.0/zephyr-sdk-0.16.0_linux-x86_64_minimal.tar.xz | tar xJ -C $HOME/.local/
curl -L https://github.com/zephyrproject-rtos/sdk-ng/releases/download/v0.16.0/toolchain_linux-x86_64_arm-zephyr-eabi.tar.xz | tar xJ -C $HOME/.local/zephyr-sdk-0.16.0/

# To run in the project folder to pull zephyr and all required libraries
tools/build/fetch_dependencies
pip install -r zephyr/scripts/requirements.txt
```

# ESB / RADIO Examples

- [nrf52-esb-broadcaster](https://github.com/NordicPlayground/nrf52-esb-broadcaster/tree/master)
- [nrf51 led_radio_example (TX)](https://github.com/finnurtorfa/nrf51/tree/master/lib/nrf51sdk/Nordic/nrf51822/Board/pca10000/led_radio_example)
- [chat GPT Code Example](https://forum.mysensors.org/topic/12188/using-chatgpt-to-write-code-for-the-nrf52840)