# isaacsim-workspace

## Installation

```bash
sudo apt update
sudo apt install libhidapi-dev
```

## USB/IP over ssh

refer to [https://qiita.com/ntrlmt/items/ec19016c2bae9b8ca3bd](https://qiita.com/ntrlmt/items/ec19016c2bae9b8ca3bd) for more information.

### Install usbip on Linux

```bash
$ sudo apt update
$ sudo apt install linux-tools-generic linux-cloud-tools-generic
$ sudo apt install linux-tools-virtual hwdata
```

```bash
$ usbip version
```

### Install usbip on Windows

Launch PowerShell as Administrator and run the following command:

```powershell
$ winget install --interactive --exact dorssel.usbipd-win
```

### Send usbipd on Windows

```powershell
$ usbpid.exe list

Connected:
BUSID  VID:PID    DEVICE                                                        STATE
2-3    054c:09cc  Wireless Controller, USB 入力デバイス                         Not shared
2-6    1bcf:28cc  Integrated Webcam                                             Not shared

$ usbipd bind -b 2-3 -f

Connected:
BUSID  VID:PID    DEVICE                                                        STATE
2-3    054c:09cc  Wireless Controller, USB 入力デバイス                         Shared (forced)
2-6    1bcf:28cc  Integrated Webcam                                             Not shared

Persisted:
GUID                                  DEVICE
f3624168-a2b0-4276-83c0-2ede598c910d  Wireless Controller, USB 入力デバイス
```

### Send usbipd on Linux

```bash
$ sudo modprobe vhci-hcd
$ sudo modprobe usbip_host
$ sudo usbip list -l

 - busid 1-10 (8087:0026)
   Intel Corp. : unknown product (8087:0026)

 - busid 1-2 (054c:09cc)
   Sony Corp. : unknown product (054c:09cc)

$ sudo usbip bind -b 1-2

usbip: info: bind device on busid 1-2: complete

$ sudo usbip -D
$ sudo usbip list

usbipd: info: starting usbipd (usbip-utils 2.0)
usbipd: info: listening on 0.0.0.0:3240
usbipd: info: listening on :::3240
usbipd: info: connection from 127.0.0.1:38046
usbipd: info: received request: 0x8005(5)
usbipd: info: exportable devices: 1
usbipd: info: request 0x8005(5): complete
```

### Connect usbipd via ssh

```bash
$ ssh -L 3240:localhost:3240 -N -f {remote-user}@{remote-addr}
```

### Receive usbipd on Linux

```bash
$ sudo modprobe vhci-hcd
$ usbip list -r localhost

Exportable USB devices
======================
 - localhost
        2-3: Sony Corp. : DualShock 4 [CUH-ZCT2x] (054c:09cc)
           : USB\VID_054C&PID_09CC\5&128B4AD4&0&3
           : (Defined at Interface level) (00/00/00)
           :  0 - Audio / Control Device / unknown protocol (01/01/00)
           :  1 - Audio / Streaming / unknown protocol (01/02/00)
           :  2 - Audio / Streaming / unknown protocol (01/02/00)
           :  3 - Human Interface Device / No Subclass / None (03/00/00)

$ sudo usbip attach -r localhost -b 2-3
$ sudo usbip port

Imported USB devices
====================
Port 00: <Port in Use> at Full Speed(12Mbps)
       Sony Corp. : DualShock 4 [CUH-ZCT2x] (054c:09cc)
       5-1 -> usbip://localhost:3240/2-3
           -> remote bus/dev 002/003

$ lsusb

Bus 005 Device 002: ID 054c:09cc Sony Corp. DualShock 4 [CUH-ZCT2x]
```
