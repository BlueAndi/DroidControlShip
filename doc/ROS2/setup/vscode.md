# VSCode in WSL

All of the following steps are applied in the Ubuntu terminal.

If DroidControlShip is not cloned yet, this will be the first step:

```bash
cd ~
git clone https://github.com/BlueAndi/DroidControlShip.git
```

In VSCode the PlatformIO extension is required. The PlatformIO extension requires Python with virtual environment capabilities.

```bash
sudo apt install -y python3-venv
```

Start VSCode inside the DroidControlShip folder.

```bash
cd DroidControlShip
vscode .
```

Install now the PlatformIO extension (extension id: platformio.platformio-ide) in VSCode and wait until installation is complete.

After that a VSCode restart is required and VSCode is ready.

Install additionally the C/C++ Extension Pack (extension id: ms-vscode.cpptools-extension-pack).
