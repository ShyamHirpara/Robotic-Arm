# Starts the Windows Mobile Hotspot that the robotic arm's Pico connects to.
# The PC stays connected to ARS_5G (internet keeps working) while broadcasting
# a 2.4 GHz hotspot for the Pico. Re-run this after a reboot if the arm is
# unreachable: powershell -ExecutionPolicy Bypass -File start_hotspot.ps1

$ErrorActionPreference = 'Stop'

Add-Type -AssemblyName System.Runtime.WindowsRuntime
$null = [Windows.Networking.Connectivity.NetworkInformation, Windows.Networking.Connectivity, ContentType = WindowsRuntime]
$null = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager, Windows.Networking.NetworkOperators, ContentType = WindowsRuntime]

$asTaskGeneric = ([System.WindowsRuntimeSystemExtensions].GetMethods() |
    Where-Object { $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and $_.GetParameters()[0].ParameterType.Name -eq 'IAsyncOperation`1' })[0]

function Await($WinRtTask, $ResultType) {
    $asTask = $asTaskGeneric.MakeGenericMethod($ResultType)
    $netTask = $asTask.Invoke($null, @($WinRtTask))
    $netTask.Wait(-1) | Out-Null
    $netTask.Result
}

function AwaitAction($WinRtAction) {
    $asTask = ([System.WindowsRuntimeSystemExtensions].GetMethods() |
        Where-Object { $_.Name -eq 'AsTask' -and $_.GetParameters().Count -eq 1 -and !$_.IsGenericMethod })[0]
    $netTask = $asTask.Invoke($null, @($WinRtAction))
    $netTask.Wait(-1) | Out-Null
}

$connectionProfile = [Windows.Networking.Connectivity.NetworkInformation]::GetInternetConnectionProfile()
if ($null -eq $connectionProfile) { throw "No internet connection profile - connect to WiFi first." }

$manager = [Windows.Networking.NetworkOperators.NetworkOperatorTetheringManager]::CreateFromConnectionProfile($connectionProfile)

$config = $manager.GetCurrentAccessPointConfiguration()
$config.Ssid = 'RoboticArm_PC'
$config.Passphrase = '12345678'
try {
    $config.Band = [Windows.Networking.NetworkOperators.TetheringWiFiBand]::TwoPointFourGigahertz
} catch {
    Write-Warning "Could not force 2.4 GHz band: $($_.Exception.Message)"
}
AwaitAction ($manager.ConfigureAccessPointAsync($config))

if ($manager.TetheringOperationalState -eq 'On') {
    Write-Output "Hotspot already on - restarting with new config"
    $null = Await ($manager.StopTetheringAsync()) ([Windows.Networking.NetworkOperators.NetworkOperatorTetheringOperationResult])
}

$result = Await ($manager.StartTetheringAsync()) ([Windows.Networking.NetworkOperators.NetworkOperatorTetheringOperationResult])
Write-Output "Start result : $($result.Status)"
Write-Output "Hotspot state: $($manager.TetheringOperationalState)"
Write-Output "SSID         : RoboticArm_PC   Password: 12345678   Band: 2.4 GHz"
