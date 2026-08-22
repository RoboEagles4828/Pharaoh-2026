cd C:\Pharaoh-2026
REM git commit
REM git push
REM WPILib: Deploy Robot Code

echo [INFO] Staging all changes...
git add .

echo [INFO] Checking for local changes...
git diff --cached --quiet
if %ERRORLEVEL% NEQ 0 (
    set HAS_CHANGES=true
    echo [INFO] Changes staged. Creating commit...
    git commit -m "Auto-commit from batch script"
    if %ERRORLEVEL% NEQ 0 (
        echo [ERROR] Failed to create commit.
        pause
        exit /b 1
    )
) else (
    echo [INFO] No changes to commit.
)

echo [INFO] Fetching latest changes from remote...
git fetch origin
if %ERRORLEVEL% NEQ 0 (
    echo [ERROR] Failed to fetch from remote.
    pause
    exit /b 1
)

REM Check for conflicts before pushing
git status --porcelain | findstr "^??" >nul
if %ERRORLEVEL% EQU 0 (
    echo [WARNING] Untracked files detected. They won't be pushed unless committed.
)

REM Try to pull first (to catch conflicts before push)
echo [INFO] Pulling latest changes to check for conflicts...
git pull --no-rebase origin %BRANCH% 2>&1 | findstr "CONFLICT"
if %ERRORLEVEL% EQU 0 (
    echo.
    echo =========================================
    echo [CONFLICT DETECTED]
    echo =========================================
    echo There are merge conflicts that need to be resolved!
    echo.
    echo To fix:
    echo  1. Manually resolve conflicts in affected files
    echo  2. Add resolved files: git add <file>
    echo  3. Complete merge: git commit
    echo  4. Run this script again to push
    echo =========================================
    git status
    pause
    exit /b 1
)

echo [INFO] Attempting to push to GitHub...
git push -u origin %BRANCH%
push_result=%ERRORLEVEL%

if %push_result% NEQ 0 (
    echo.
    echo =========================================
    echo [PUSH FAILED]
    echo =========================================
    echo Error code: %push_result%
    echo.
    echo Common causes:
    echo  - Remote branch has commits you don't have (try: git pull first)
    echo  - Authentication failure (check credentials/token)
    echo  - Network connection issue
    echo  - Repository permissions problem
    echo =========================================
    pause
    exit /b 1
)

echo.
echo ================================
echo [SUCCESS] Successfully pushed to GitHub!
echo ================================

WPILib: Deploy Robot Code