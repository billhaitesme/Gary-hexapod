<#
.SYNOPSIS
    Gary WonderEcho substitution script — V2.2.0 SDK target.

.DESCRIPTION
    Adapted for the older CI130X_SDK_ASR_Offline_V2.2.0 SDK which:
      - Has prebuilt user_code via libs/libfbin_pro.a (no compiler needed)
      - Uses ci-tool-kit 1.20 which requires the cmd_info_old.xlsx format
      - Has project name 'offline_asr_pro_sample' (no '_alg_')

    Operations:
      1. Validates paths and detects nested SDK folder
      2. Backs up firmware/asr/, voice/src/, user_file/cmd_info/
      3. Extracts both ChipIntelli zips to a temp directory
      4. Replaces stock LM .dat files
      5. Replaces stock TTS WAVs
      6. Replaces stock cmd_info xlsx with the OLD-format version
      7. Pre-creates user_code/ directory (combine.bat will populate it)
      8. Reports counts for verification

    To roll back: copy contents of any _backup_<timestamp> folder back over.
#>

param(
    [string]$SdkRoot = "C:\Users\BillH\Documents\Arduino\gary_v1\CI130X_SDK_ASR_Offline_V2.2.0",
    [string]$LmZip   = "C:\Users\BillH\Downloads\lm20260428204312141568066.zip",
    [string]$TtsZip  = "C:\Users\BillH\Downloads\tts20260428205638141561339.zip",
    [switch]$DryRun
)

$ErrorActionPreference = "Stop"

function Write-Section($text) {
    Write-Host ""
    Write-Host "=== $text ===" -ForegroundColor Cyan
}
function Write-OK($text)   { Write-Host "  [OK]   $text" -ForegroundColor Green }
function Write-Warn($text) { Write-Host "  [WARN] $text" -ForegroundColor Yellow }
function Write-Err($text)  { Write-Host "  [FAIL] $text" -ForegroundColor Red }
function Write-Info($text) { Write-Host "  $text" -ForegroundColor Gray }

# ---- Validate inputs ---------------------------------------------------------
Write-Section "Validating paths"

foreach ($pair in @(
    @{ Name = "SDK root";    Path = $SdkRoot },
    @{ Name = "LM zip";      Path = $LmZip   },
    @{ Name = "TTS zip";     Path = $TtsZip  }
)) {
    if (Test-Path -LiteralPath $pair.Path) {
        Write-OK "$($pair.Name): $($pair.Path)"
    } else {
        Write-Err "$($pair.Name) NOT FOUND: $($pair.Path)"
        exit 1
    }
}

# Auto-detect nested SDK folder. The real SDK has a 'projects' subdir.
$ProjectsDir = Join-Path $SdkRoot "projects"
if (-not (Test-Path -LiteralPath $ProjectsDir)) {
    $candidate = Get-ChildItem -LiteralPath $SdkRoot -Directory | Where-Object { Test-Path (Join-Path $_.FullName "projects") } | Select-Object -First 1
    if ($candidate) {
        $SdkRoot = $candidate.FullName
        Write-Info "Auto-detected nested SDK at: $SdkRoot"
        $ProjectsDir = Join-Path $SdkRoot "projects"
    } else {
        Write-Err "Could not locate 'projects' folder under SDK root."
        exit 1
    }
}

# Confirm libfbin_pro.a exists (the prebuilt user_code blob)
$LibFbinPath = Join-Path $SdkRoot "libs\libfbin_pro.a"
if (-not (Test-Path -LiteralPath $LibFbinPath)) {
    Write-Err "libs/libfbin_pro.a NOT FOUND at $LibFbinPath - V2.2.0 SDK is incomplete"
    exit 1
}
Write-OK "libfbin_pro.a present (user_code prebuilt blob ready)"

$FwRoot       = Join-Path $SdkRoot "projects\offline_asr_pro_sample\firmware"
$AsrDir       = Join-Path $FwRoot "asr"
$VoiceSrcDir  = Join-Path $FwRoot "voice\src"
$CmdInfoDir   = Join-Path $FwRoot "user_file\cmd_info"
$UserCodeDir  = Join-Path $FwRoot "user_code"

foreach ($pair in @(
    @{ Name = "firmware/asr";                Path = $AsrDir      },
    @{ Name = "firmware/voice/src";          Path = $VoiceSrcDir },
    @{ Name = "firmware/user_file/cmd_info"; Path = $CmdInfoDir  }
)) {
    if (Test-Path -LiteralPath $pair.Path) {
        Write-OK "$($pair.Name): $($pair.Path)"
    } else {
        Write-Err "$($pair.Name) NOT FOUND: $($pair.Path)"
        exit 1
    }
}

# ---- Show plan and confirm ---------------------------------------------------
Write-Section "Plan"
Write-Host "  Will REPLACE all .dat files in:    $AsrDir"
Write-Host "  Will REPLACE all .wav files in:    $VoiceSrcDir"
Write-Host "  Will REPLACE [60000]*.xlsx in:     $CmdInfoDir"
Write-Host "  Will CREATE (if missing):          $UserCodeDir"
Write-Host ""
Write-Host "  Backup will be saved to:           $FwRoot\_backup_<timestamp>"

if ($DryRun) {
    Write-Host ""
    Write-Host "  *** DRY RUN MODE - no files will actually change ***" -ForegroundColor Yellow
} else {
    Write-Host ""
    $resp = Read-Host "Proceed? (type 'yes' to continue)"
    if ($resp -ne "yes") {
        Write-Host "Aborted by user." -ForegroundColor Yellow
        exit 0
    }
}

# ---- Extract zips ------------------------------------------------------------
Write-Section "Extracting zips"

$Stamp   = Get-Date -Format "yyyyMMdd_HHmmss"
$TempDir = Join-Path $env:TEMP "gary_substitution_v22_$Stamp"
$LmTemp  = Join-Path $TempDir "lm"
$TtsTemp = Join-Path $TempDir "tts"

if (-not $DryRun) {
    New-Item -ItemType Directory -Path $TempDir -Force | Out-Null
    Expand-Archive -LiteralPath $LmZip  -DestinationPath $LmTemp  -Force
    Expand-Archive -LiteralPath $TtsZip -DestinationPath $TtsTemp -Force
}

function Find-InnerFolder($parent) {
    if ($DryRun) { return $null }
    $inner = Get-ChildItem -LiteralPath $parent -Directory | Select-Object -First 1
    if (-not $inner) { Write-Err "No inner folder under $parent"; exit 1 }
    return $inner.FullName
}
$LmInner  = Find-InnerFolder $LmTemp
$TtsInner = Find-InnerFolder $TtsTemp

if (-not $DryRun) {
    Write-OK "LM:  $LmInner"
    Write-OK "TTS: $TtsInner"
}

# ---- Backup ------------------------------------------------------------------
Write-Section "Backing up current firmware"

$BackupRoot = Join-Path $FwRoot "_backup_$Stamp"
if (-not $DryRun) {
    New-Item -ItemType Directory -Path $BackupRoot -Force | Out-Null
    Copy-Item -LiteralPath $AsrDir      -Destination (Join-Path $BackupRoot "asr")      -Recurse
    Copy-Item -LiteralPath $VoiceSrcDir -Destination (Join-Path $BackupRoot "src")      -Recurse
    Copy-Item -LiteralPath $CmdInfoDir  -Destination (Join-Path $BackupRoot "cmd_info") -Recurse
    Write-OK "Backup at: $BackupRoot"
} else {
    Write-Info "(would back up to: $BackupRoot)"
}

# ---- Substitute asr/ ---------------------------------------------------------
Write-Section "Replacing asr/ (LM .dat files)"

if (-not $DryRun) {
    Get-ChildItem -LiteralPath $AsrDir -Filter "*.dat" | ForEach-Object {
        Write-Info "  - removing $($_.Name)"
        Remove-Item -LiteralPath $_.FullName -Force
    }

    $NewLmCmd = Get-ChildItem -LiteralPath (Join-Path $LmInner "GfstCmd")  -Filter "*.dat" | Select-Object -First 1
    $NewLmWk  = Get-ChildItem -LiteralPath (Join-Path $LmInner "GfstWake") -Filter "*.dat" | Select-Object -First 1
    if (-not $NewLmCmd) { Write-Err "No .dat in GfstCmd"; exit 1 }
    if (-not $NewLmWk)  { Write-Err "No .dat in GfstWake"; exit 1 }
    Copy-Item -LiteralPath $NewLmCmd.FullName -Destination $AsrDir
    Copy-Item -LiteralPath $NewLmWk.FullName  -Destination $AsrDir
    Write-OK "  + copied $($NewLmCmd.Name)"
    Write-OK "  + copied $($NewLmWk.Name)"

    $count = (Get-ChildItem -LiteralPath $AsrDir -Filter "*.dat").Count
    if ($count -eq 2) { Write-OK "asr/ has 2 .dat files (correct)" }
    else              { Write-Warn "asr/ has $count .dat files (expected 2)" }
}

# ---- Substitute voice/src/ ---------------------------------------------------
Write-Section "Replacing voice/src/ (TTS WAVs)"

if (-not $DryRun) {
    Get-ChildItem -LiteralPath $VoiceSrcDir -Filter "*.wav" | ForEach-Object {
        Write-Info "  - removing $($_.Name)"
        Remove-Item -LiteralPath $_.FullName -Force
    }

    $TtsSrc = Join-Path $TtsInner "src"
    $newWavs = Get-ChildItem -LiteralPath $TtsSrc -Filter "*.wav"
    foreach ($wav in $newWavs) {
        Copy-Item -LiteralPath $wav.FullName -Destination $VoiceSrcDir
    }
    Write-OK "  + copied $($newWavs.Count) .wav files"

    $count = (Get-ChildItem -LiteralPath $VoiceSrcDir -Filter "*.wav").Count
    Write-OK "voice/src/ now contains $count .wav files"
}

# ---- Substitute user_file/cmd_info/ ------------------------------------------
Write-Section "Replacing user_file/cmd_info/ (using OLD format for ci-tool-kit 1.20)"

if (-not $DryRun) {
    # Delete only existing xlsx (and any leftover .bin from previous runs); keep .bat and .doc
    Get-ChildItem -LiteralPath $CmdInfoDir | Where-Object { $_.Extension -in '.xlsx','.bin' } | ForEach-Object {
        Write-Info "  - removing $($_.Name)"
        Remove-Item -LiteralPath $_.FullName -Force
    }

    # Find the OLD-format cmd_info — V2.2.0 + ci-tool-kit 1.20 needs this one
    $NewCmdInfo = Get-ChildItem -LiteralPath (Join-Path $LmInner "CmdWordStructure") -Filter "*.xlsx" |
        Where-Object { $_.Name -like "*cmd_info_old*" } |
        Select-Object -First 1

    if (-not $NewCmdInfo) { Write-Err "No cmd_info_old.xlsx found in CmdWordStructure"; exit 1 }
    Copy-Item -LiteralPath $NewCmdInfo.FullName -Destination $CmdInfoDir
    Write-OK "  + copied $($NewCmdInfo.Name) (OLD/V2 format)"
}

# ---- Pre-create user_code/ directory -----------------------------------------
Write-Section "Preparing user_code/ directory"

if (-not $DryRun) {
    if (-not (Test-Path -LiteralPath $UserCodeDir)) {
        New-Item -ItemType Directory -Path $UserCodeDir -Force | Out-Null
        Write-OK "Created $UserCodeDir"
    } else {
        Write-OK "$UserCodeDir already exists"
    }
    Write-Info "(combine.bat will populate this from libfbin_pro.a automatically)"
}

# ---- Done --------------------------------------------------------------------
Write-Section "Substitution complete"

Write-Host "Backup location: $BackupRoot"
Write-Host ""
Write-Host "Next step: cd into the firmware folder and run combine.bat" -ForegroundColor Cyan
Write-Host "  cd `"$FwRoot`""
Write-Host "  cmd /c `"合成分区bin文件.bat`" *>&1 | Tee-Object -FilePath combine_log.txt"
Write-Host ""
Write-Host "Then we'll move on to the packaging + flash steps."
