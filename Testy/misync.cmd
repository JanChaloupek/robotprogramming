@echo off 
setlocal enabledelayedexpansion

set help_flag=0
set quick_flag=0
set clean_flag=0

rem Process params from CLI
:CheckParams
if "%1"=="" goto run

if "%1"=="h" set help_flag=1
if "%1"=="help" set help_flag=1
if "%1"=="/help" set help_flag=1
if "%1"=="/?" set help_flag=1
if "%1"=="--help" set help_flag=1

if "%1"=="/quick" set quick_flag=1
if "%1"=="--quick" set quick_flag=1
if "%1"=="/q" set quick_flag=1
if "%1"=="q" set quick_flag=1

if "%1"=="/clean" set clean_flag=1
if "%1"=="--clean" set clean_flag=1
if "%1"=="/c" set clean_flag=1
if "%1"=="c" set clean_flag=1

shift
goto CheckParams

:run

if %help_flag% EQU 1 (
	goto printHelp
)

rem Clean all files in microbit
(
	if %clean_flag% EQU 1 (
		set "filesInMicrobit="
		echo Clean microbit
	    for /f "tokens=*" %%A in ('ufs.exe ls') do set "filesInMicrobit=!filesInMicrobit!, %%A"
	    
	   	for %%A in (!filesInMicrobit!) do (
	    	echo|set /p=" # %%A .... "
	    	ufs.exe rm %%A
	    	echo [OK]
	    )

		exit /b 0
	)
)

rem Prepare folders
(
	if not exist dist (
		mkdir dist
		echo Dist folder created
	)
	cd dist
	if exist transfer rmdir transfer /S /Q
	mkdir transfer
	if not exist temp mkdir temp
	if not exist sha1 mkdir sha1
	cd ..
)

rem Prepare files to tranfer to microbit
(
	set pocet=0
	for %%a in (*.py) do set /a pocet+=1

	echo.
	echo Prepare file for transfer: !pocet!

	for %%f in (*.py) do (

		certutil -hashfile %%f SHA1 > dist/temp/sha1.txt

		set "counter=0"
		for /f "tokens=*" %%a in (dist/temp/sha1.txt) do (
			set "transferFile=0"

			if !quick_flag! EQU 0 (
				set /a transferFile=1
			)

			set /a counter+=1
		    if !counter! equ 2 (

		    	if not exist dist/sha1/%%f (
		    		set /a transferFile=1
		    	) else (
					for /f "delims=" %%b in (dist/sha1/%%f) do (
						if %%a neq %%b (
							set /a transferFile=1
						)
					)
		    	)
		    	echo|set /p=" # %%f ...."
			    if !transferFile! equ 1 (
					echo %%a>dist/sha1/%%f
					pyminify %%f --output ./dist/transfer/%%f >NUL
					echo  [Minimized]
				) else (
					echo  [Skipped]
				)
		    )
		)
	)
)

rem Transfer files to microbit
(
	set /a pocet=0
	for %%a in (dist/transfer/*.py) do set /a pocet+=1

	echo.
	echo Transfer files: %pocet%

	(for %%f in (dist/transfer/*.py) do (
		echo|set /p=" # %%f ...."
		
		ufs.exe put ./dist/transfer/%%f>dist/temp/%%f
		for %%I in (dist/temp/%%f) do set /a size=%%~zI
		
		if !size! NEQ 0 (
			echo  [ERROR]
		) else (
			echo  [OK]
		)
	))
)
:retry
(
	for %%f in (dist/temp/*.py) do (
		for %%I in (dist/temp/%%f) do set /a size=%%~zI
		
		if !size! NEQ 0 (
			echo|set /p=" # %%f ...."
			ufs.exe put ./dist/transfer/%%f>dist/temp/%%f
			for %%I in (dist/temp/%%f) do set /a size=%%~zI
			
			if !size! NEQ 0 (
				echo  [ERROR]
				goto retry
			) else (
				echo  [OK]
			)
		)
	)
)

exit /b 0

:printHelp
(
	echo Help for Microbit syncer
	echo.
	echo Parametry
	echo  /q /quick    Zrychlene nahravani.
	echo  /c /clean    Vymaze pamet mikrobitu
	echo  /h /help     Tato napoveda
	exit /b 0
)
