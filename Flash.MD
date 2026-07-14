# Flash and UART Notes

This project was verified on 2026-07-14 with the board connected as COM10.

## Build

Use Keil uVision command-line wrapper:

```powershell
& 'D:\Keil5\UV4\uVision.com' -b 'D:\Keil5\Project\MPU6050Test\keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx' -t 'MSPM0G3507_Project' -o 'D:\Keil5\Project\MPU6050Test\keil\build_log_new.txt'
```

Expected result:

```text
0 Error(s), 0 Warning(s)
```

Generated output:

```text
D:\Keil5\Project\MPU6050Test\keil\Objects\empty_LP_MSPM0G3507_nortos_keil.axf
D:\Keil5\Project\MPU6050Test\keil\Objects\empty_LP_MSPM0G3507_nortos_keil.hex
```

## Flash

Use the same Keil command-line wrapper for CMSIS-DAP flashing:

```powershell
& 'D:\Keil5\UV4\uVision.com' -f 'D:\Keil5\Project\MPU6050Test\keil\empty_LP_MSPM0G3507_nortos_keil.uvprojx' -t 'MSPM0G3507_Project' -o 'D:\Keil5\Project\MPU6050Test\keil\flash_log_new.txt'
```

Expected result:

```text
Load "D:\Keil5\Project\MPU6050Test\keil\Objects\empty_LP_MSPM0G3507_nortos_keil.axf"
Erase Done.
Programming Done.
Verify OK.
Application running ...
```

Important: use `D:\Keil5\UV4\uVision.com` for command-line operations. `UV4.exe` may attach to an existing GUI instance and may not refresh the requested log file.

## UART Monitor

The board's USB serial device was detected as:

```text
COM10    USB Serial Device (COM10)
```

UART0 settings:

```text
Port: COM10
Baud: 9600
Data bits: 8
Parity: None
Stop bits: 1
Flow control: None
TX pin: PA10
RX pin: PA11
```

PowerShell quick read:

```powershell
$port = New-Object System.IO.Ports.SerialPort 'COM10', 9600, 'None', 8, 'One'
$port.DtrEnable = $true
$port.RtsEnable = $true
$port.ReadTimeout = 500
$port.Open()
$deadline = (Get-Date).AddSeconds(10)
while ((Get-Date) -lt $deadline) {
    $text = $port.ReadExisting()
    if ($text.Length -gt 0) { Write-Output $text }
    Start-Sleep -Milliseconds 100
}
$port.Close()
```

Expected UART output:

```text
MPU6050 R=-0.1 P=-0.7 Y=+0.0 T=+47.8C ACC=166,-141,16379 GYRO=0,0,0
```

If MPU6050 initialization fails, firmware prints repeatedly:

```text
MPU6050 init failed
```

If MPU6050 read fails after initialization, firmware prints periodically:

```text
MPU6050 read failed
```
