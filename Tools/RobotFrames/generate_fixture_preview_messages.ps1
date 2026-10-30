param([string]$ProjectRoot = (Join-Path $PSScriptRoot '../..'))
$ErrorActionPreference = 'Stop'
$taskProject = [IO.Path]::GetFullPath($ProjectRoot)
$taskConnector = @(Get-ChildItem -LiteralPath (Join-Path $taskProject 'Library/PackageCache') -Directory |
    Where-Object Name -Like 'com.unity.robotics.ros-tcp-connector@*')
if ($taskConnector.Count -ne 1) { throw 'Expected one cached ROS TCP Connector package.' }
$taskGeneratorRoot = Join-Path $taskConnector[0].FullName 'Editor/MessageGeneration'
$taskGeneratorFiles = @('MessageAutoGen.cs', 'MessageParser.cs', 'MessageTokenizer.cs', 'MessageToken.cs', 'MsgAutoGenUtilities.cs')
$taskGeneratorSources = @($taskGeneratorFiles | ForEach-Object { Join-Path $taskGeneratorRoot $_ })
$taskGeneratorSources += Join-Path $taskConnector[0].FullName 'Runtime/MessageGeneration/MessageSubtopic.cs'
# Only warning output needs UnityEngine when the official generator runs outside Editor.
$taskSupport = Join-Path $taskProject 'Library/FixtureTilingValidationSupport'
New-Item -ItemType Directory -Path $taskSupport -Force | Out-Null
$taskWarningStub = Join-Path $taskSupport 'GeneratorWarningStub.cs'
'namespace UnityEngine { public static class Debug { public static void LogWarning(object value) { System.Console.Error.WriteLine(value); } public static void LogWarningFormat(string value, params object[] args) { System.Console.Error.WriteLine(value, args); } } }' | Set-Content -LiteralPath $taskWarningStub
$taskGeneratorSources += $taskWarningStub
if (-not ('Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageAutoGen' -as [type])) {
    Add-Type -Path $taskGeneratorSources
}
$taskManifest = Get-Content -LiteralPath (Join-Path $PSScriptRoot 'RosMessageSchemas/manifest.json') -Raw | ConvertFrom-Json
$taskOutput = Join-Path $taskProject 'Assets/RosMessages'
$taskTemporaryOutput = Join-Path $taskSupport 'GeneratedMessages'
$taskGeneratedCount = 0
foreach ($taskMessage in $taskManifest.messages) {
    if (-not $taskMessage.generate) { continue }
    $taskSchema = Join-Path $PSScriptRoot ('RosMessageSchemas/' + $taskMessage.package + '/msg/' + $taskMessage.name + '.msg')
    if ((Get-FileHash -LiteralPath $taskSchema -Algorithm SHA256).Hash.ToLowerInvariant() -ne $taskMessage.sha256) {
        throw ('Schema hash mismatch: ' + $taskSchema)
    }
    [Unity.Robotics.ROSTCPConnector.MessageGeneration.MessageAutoGen]::GenerateSingleMessage($taskSchema, $taskTemporaryOutput, $taskMessage.package, $false) | Write-Output
    $taskNamespace = [Unity.Robotics.ROSTCPConnector.MessageGeneration.MsgAutoGenUtilities]::ResolvePackageName($taskMessage.package)
    $taskGeneratedPath = Join-Path $taskTemporaryOutput ($taskNamespace + '/msg/' + $taskMessage.name + 'Msg.cs')
    $taskSource = Get-Content -LiteralPath $taskGeneratedPath -Raw
    foreach ($taskReplacement in $taskManifest.messages | Where-Object privateNamespace) {
        $taskOldNamespace = [Unity.Robotics.ROSTCPConnector.MessageGeneration.MsgAutoGenUtilities]::ResolvePackageName($taskReplacement.package)
        $taskSource = $taskSource.Replace(($taskOldNamespace + '.' + $taskReplacement.name + 'Msg'), ($taskReplacement.privateNamespace + '.' + $taskReplacement.name + 'Msg'))
    }
    $taskDestinationNamespace = $taskNamespace
    if ($taskMessage.privateNamespace) {
        $taskDestinationNamespace = $taskMessage.privateNamespace
        $taskSource = $taskSource.Replace(('namespace RosMessageTypes.' + $taskNamespace), ('namespace RosMessageTypes.' + $taskDestinationNamespace))
        # Private nested serializers must not replace connector-wide message registrations.
        $taskSource = [regex]::Replace($taskSource, '(?s)\n#if UNITY_EDITOR\n.*?\n        }\n', "`n")
    }
    $taskDestinationPath = Join-Path $taskOutput ($taskDestinationNamespace + '/msg/' + $taskMessage.name + 'Msg.cs')
    New-Item -ItemType Directory -Path ([IO.Path]::GetDirectoryName($taskDestinationPath)) -Force | Out-Null
    [IO.File]::WriteAllText($taskDestinationPath, $taskSource)
    $taskGeneratedCount++
}
Get-ChildItem -LiteralPath $taskOutput -Recurse -Filter '*.cs' | ForEach-Object {
    $taskMeta = $_.FullName + '.meta'
    if (-not (Test-Path -LiteralPath $taskMeta)) {
        # Stable GUID derived from the project-relative source path.
        $taskRelative = $_.FullName.Substring($taskProject.Length + 1).Replace('\', '/')
        $taskHash = [Security.Cryptography.MD5]::HashData([Text.Encoding]::UTF8.GetBytes($taskRelative))
        $taskGuid = [Convert]::ToHexString($taskHash).ToLowerInvariant()
        @('fileFormatVersion: 2', ('guid: ' + $taskGuid)) | Set-Content -LiteralPath $taskMeta -Encoding utf8
    }
}
Write-Output ('Generated fixture preview ROS closure: ' + $taskGeneratedCount + ' message types; compatible standard types use the connector assembly.')
