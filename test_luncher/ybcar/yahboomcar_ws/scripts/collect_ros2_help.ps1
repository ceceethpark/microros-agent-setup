Param(
    [switch]$Run
)

# PowerShell wrapper to run the Python helper. Requires Python and a sourced ROS2 environment.
$py = Join-Path $PSScriptRoot 'collect_ros2_help.py'
if (-not (Test-Path $py)) {
    Write-Error "Missing $py"
    exit 1
}
if ($Run) {
    python3 $py --run
} else {
    python3 $py
}
