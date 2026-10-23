param([string]$ProjectRoot = (Join-Path $PSScriptRoot '../..'))
$ErrorActionPreference = 'Stop'
$taskProject = [IO.Path]::GetFullPath($ProjectRoot)
$taskUnityData = 'C:/Program Files/Unity 6000.0.84f1/Editor/Data'
$taskDotnet = Join-Path $taskUnityData 'NetCoreRuntime/dotnet.exe'
$taskCompiler = Join-Path $taskUnityData 'DotNetSdkRoslyn/csc.dll'
$taskResponseRelative = 'Library/Bee/artifacts/1900b0aE.dag/Assembly-CSharp.rsp'
$taskOutputDirectory = Join-Path ([IO.Path]::GetTempPath()) ('unity-ros-compile-' + [guid]::NewGuid().ToString('N'))
New-Item -ItemType Directory -Path $taskOutputDirectory | Out-Null
Push-Location $taskProject
try {
    $taskOptions = @(Get-Content $taskResponseRelative | Where-Object { $_ -notmatch '^".*\.cs"$' -and $_ -notmatch '^[-/](out|refout|pdb):' })
    $taskFrameSources = @(rg --files --no-ignore Assets/Scripts/V3/Frames -g '*.cs' | ForEach-Object { $_.Replace('\','/') })
    $taskScopedDirectories = @(rg --files --no-ignore Assets -g '*.asmdef' -g '*.asmref' | ForEach-Object { ([IO.Path]::GetDirectoryName($_)).Replace('\','/')+'/' })
    $taskMainSources = @(rg --files --no-ignore Assets -g '*.cs' | ForEach-Object { $_.Replace('\','/') } | Where-Object {
        $taskSource = $_
        $taskSource -notmatch '/(Editor|Tests)/' -and -not ($taskScopedDirectories | Where-Object { $taskSource.StartsWith($_) })
    })
    $taskSources = @($taskFrameSources + $taskMainSources | Sort-Object -Unique)
    $taskHashes = @($taskSources | ForEach-Object { @{ path = $_; sha256 = (Get-FileHash -LiteralPath $_ -Algorithm SHA256).Hash.ToLowerInvariant() } })
    $taskFrameRef = Join-Path $taskOutputDirectory 'UnityROSML.RobotFrames.ref.dll'
    $taskFrameResponse = Join-Path $taskOutputDirectory 'frames.rsp'
    @($taskOptions | Where-Object { $_ -notmatch 'ROSTCPConnector|Interaction.Toolkit.Samples|UnityAdditionalFile' }) +
        @('-out:"'+(Join-Path $taskOutputDirectory 'UnityROSML.RobotFrames.dll')+'"','-refout:"'+$taskFrameRef+'"') +
        @($taskFrameSources | ForEach-Object { '"'+$_+'"' }) | Set-Content -LiteralPath $taskFrameResponse -Encoding utf8
    $taskDiagnostics = @(& $taskDotnet exec $taskCompiler /noconfig /nostdlib ('@'+$taskFrameResponse) 2>&1 | ForEach-Object { $_.ToString() })
    if ($LASTEXITCODE -ne 0) { $taskDiagnostics | Write-Output; throw 'RobotFrames compilation failed.' }
    $taskMainResponse = Join-Path $taskOutputDirectory 'main.rsp'
    @($taskOptions) + @('-r:"'+$taskFrameRef+'"','-out:"'+(Join-Path $taskOutputDirectory 'Assembly-CSharp.dll')+'"') +
        @($taskMainSources | ForEach-Object { '"'+$_+'"' }) | Set-Content -LiteralPath $taskMainResponse -Encoding utf8
    $taskDiagnostics += @(& $taskDotnet exec $taskCompiler /noconfig /nostdlib ('@'+$taskMainResponse) 2>&1 | ForEach-Object { $_.ToString() })
    if ($LASTEXITCODE -ne 0) { $taskDiagnostics | Write-Output; throw 'Main runtime compilation failed.' }
    foreach ($taskHash in $taskHashes) {
        if ((Get-FileHash -LiteralPath $taskHash.path -Algorithm SHA256).Hash.ToLowerInvariant() -ne $taskHash.sha256) {
            throw ('Source changed during compilation; rerun: '+$taskHash.path)
        }
    }
    $taskSummary = [ordered]@{
        schemaVersion = 1; completedUtc = [DateTime]::UtcNow.ToString('o'); unityVersion = '6000.0.84f1'
        command = 'powershell -NoProfile -File Tools/RobotFrames/compile_runtime.ps1'
        compiler = $taskCompiler; compilerInvocation = 'dotnet exec csc.dll /noconfig /nostdlib @response.rsp'
        settingsResponseFile = $taskResponseRelative; responseSha256 = (Get-FileHash $taskResponseRelative -Algorithm SHA256).Hash.ToLowerInvariant()
        helperSourceCount = $taskFrameSources.Count; mainSourceCount = $taskMainSources.Count; exitCode = 0
        errorCount = 0; warnings = @($taskDiagnostics | Where-Object { $_ -match 'warning CS' })
        ros2 = [bool]($taskOptions | Where-Object { $_ -match '^-define:.*\bROS2\b' })
        validatedSources = $taskHashes
        scope = 'Current project runtime sources compiled against installed Unity/Bee package references; outputs only in temporary storage.'
        hardwareExercised = $false; networkExercised = $false
    }
    $taskSummary | ConvertTo-Json -Depth 6 | Set-Content -LiteralPath (Join-Path $PSScriptRoot 'runtime_compile_summary.json') -Encoding utf8
    $taskDiagnostics | Write-Output
    Write-Output ('Compiled helper '+$taskFrameSources.Count+' and main '+$taskMainSources.Count+' sources; zero errors.')
}
finally { Pop-Location }
