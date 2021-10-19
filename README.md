# STM32H7
STM32H7 project demo.

# Getting Started
由于STM32H743VI使用非原生配置，改用Cortex-Debug+pyOCD编译调试
## .vscode/launch.json (optional)
```
{
  "version": "0.2.0",
  "configurations": [
    {
      "name": "pyOCD",
      "type": "cortex-debug",
      "request": "launch",
      "cwd": "${workspaceFolder}",
      "servertype": "pyocd",
      "executable": ".pio/build/stm32h743vit6/firmware.elf",
      "runToMain": true,
      "svdFile": "C:/Users/<username>/.platformio/platforms/ststm32/misc/svd/STM32H743.svd",
      "targetId": "STM32H743VITx",
      "cmsisPack": "C:/Users/<username>/AppData/Local/Arm/Packs/.Download/Keil.STM32H7xx_DFP.2.8.0.pack"
    }
  ]
}
```

## .vscode/tasks.json (optional)
```
{
  "version": "2.0.0",
  "tasks": [
    {
      "label": "Load",
      "type": "shell",
      "command": "pyocd",
      "args": [
        "flash",
        "--erase",
        "sector",
        "--target",
        "STM32H743VITx",
        "--pack",
        "C:/Users/<username>/AppData/Local/Arm/Packs/.Download/Keil.STM32H7xx_DFP.2.8.0.pack",
        ".pio/build/stm32h743vit6/firmware.bin"
      ],
      "group": {
        "kind": "build",
        "isDefault": true
      }
    }
  ]
}
```
