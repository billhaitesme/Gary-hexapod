<#
.SYNOPSIS
    Gary WonderEcho firmware substitution script.

.DESCRIPTION
    Replaces the stock Chinese demo content in the CI13XX SDK firmware tree
    with our custom Gary build outputs from ChipIntelli (LM .dat files,
    cmd_info.xlsx, and TTS WAVs).

    Operations:
      1. Validates all paths and detects nested SDK folder
      2. Backs up firmware/asr/, firmware/voice/src/, firmware/user_file/cmd_info/
         to firmware/_backup_<timestamp>/
      3. Extracts both zips to a temp directory
      4. Replaces stock LM .dat files with Gary's English ones
      5. Replaces stock TTS WAVs with Gary's
      6. Replaces stock cmd_info.xlsx with Gary's
      7. Reports counts for verification

    Safe to re-run: backups are kept with timestamps. To roll back, copy contents
    of any _backup_<timestamp> folder back over asr/, voice/src/, cmd_info/.

.PARAMETER SdkRoot
    Path to the SDK root (or its parent if double-nested from the zip extraction).

.PARAMETER LmZip
    Path to the language model zip from ChipIntelli (lm*.zip).

.PARAMETER TtsZip
    Path to the TTS zip from ChipIntelli (tts*.zip).

.PARAMETER DryRun
    If specified, shows what would happen but doesn't modify any files.
#>

param(
    [string]$SdkRoot = "C:\Users\BillH\Documents\Arduino\gary_v1\CI13XX_SDK_ASR_ALG_V2.7.12",
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

# ---- Validate inputs ----------------------------------------------------------
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
    # Look one level deeper
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

$FwRoot      = Join-Path $SdkRoot "projects\offline_asr_alg_pro_sample\firmware"
$AsrDir      = Join-Path $FwRoot "asr"
$VoiceSrcDir = Join-Path $FwRoot "voice\src"
$CmdInfoDir  = Join-Path $FwRoot "user_file\cmd_info"

foreach ($pair in @(
    @{ Name = "firmware/asr";              Path = $AsrDir      },
    @{ Name = "firmware/voice/src";        Path = $VoiceSrcDir },
    @{ Name = "firmware/user_file/cmd_info"; Path = $CmdInfoDir }
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
Write-Host "  Will REPLACE [60000]cmd_info.xlsx in: $CmdInfoDir"
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
$TempDir = Join-Path $env:TEMP "gary_substitution_$Stamp"
$LmTemp  = Join-Path $TempDir "lm"
$TtsTemp = Join-Path $TempDir "tts"

if (-not $DryRun) {
    New-Item -ItemType Directory -Path $TempDir -Force | Out-Null
    Expand-Archive -LiteralPath $LmZip  -DestinationPath $LmTemp  -Force
    Expand-Archive -LiteralPath $TtsZip -DestinationPath $TtsTemp -Force
}

# Find inner extracted folders (each zip contains a single folder)
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
    # Delete existing .dat files
    Get-ChildItem -LiteralPath $AsrDir -Filter "*.dat" | ForEach-Object {
        Write-Info "  - removing $($_.Name)"
        Remove-Item -LiteralPath $_.FullName -Force
    }

    # Copy our new ones
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
Write-Section "Replacing user_file/cmd_info/cmd_info.xlsx"

if (-not $DryRun) {
    # Delete only the existing cmd_info xlsx (not the .bat or .docx)
    Get-ChildItem -LiteralPath $CmdInfoDir -Filter "*.xlsx" | ForEach-Object {
        Write-Info "  - removing $($_.Name)"
        Remove-Item -LiteralPath $_.FullName -Force
    }

    # Find the new cmd_info — skip _old (V2.7.X uses the new format) and skip template
    $NewCmdInfo = Get-ChildItem -LiteralPath (Join-Path $LmInner "CmdWordStructure") -Filter "*.xlsx" |
        Where-Object { $_.Name -like "*cmd_info*" -and $_.Name -notlike "*old*" -and $_.Name -notlike "*template*" } |
        Select-Object -First 1

    if (-not $NewCmdInfo) { Write-Err "No cmd_info.xlsx found in CmdWordStructure"; exit 1 }
    Copy-Item -LiteralPath $NewCmdInfo.FullName -Destination $CmdInfoDir
    Write-OK "  + copied $($NewCmdInfo.Name)"
}

# ---- Done --------------------------------------------------------------------
Write-Section "Substitution complete"

Write-Host "Backup location: $BackupRoot"
Write-Host ""
Write-Host "Next step: open Windows Explorer to:" -ForegroundColor Cyan
Write-Host "  $FwRoot"
Write-Host ""
Write-Host "Then double-click: 合成分区bin文件.bat" -ForegroundColor Cyan
Write-Host "  (this is the 'Combine bin' batch file that builds the flashable .bin)"
Write-Host ""
Write-Host "Wait for it to finish, then we'll move on to the flash step."
