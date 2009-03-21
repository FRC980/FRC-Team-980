cd C:\WindRiver\vxworks-6.3\target

mkdir h\WPILib
mkdir h\WPILib\ChipObject
mkdir h\WPILib\CInterfaces
mkdir h\WPILib\NetworkCommunication
mkdir h\WPILib\visa

del h\WPILib\*.h
del h\WPILib\ChipObject\*.h
del h\WPILib\CInterfaces\*.h
del h\WPILib\NetworkCommunication\*.h
del h\WPILib\visa\*.h

copy c:\WindRiver\workspace\WPILib\*.h h\WPILib
copy c:\WindRiver\workspace\WPILib\ChipObject\*.h h\WPILib\ChipObject
copy C:\WindRiver\workspace\WPILib\CInterfaces\*.h h\WPILib\CInterfaces
copy C:\WindRiver\workspace\WPILib\NetworkCommunication\*.h h\WPILib\NetworkCommunication
copy c:\WindRiver\workspace\WPILib\visa\*.h h\WPILib\visa

copy C:\WindRiver\workspace\WPILib\PPC603gnu_DEBUG\WPILib.a lib

cd \WindRiver\workspace\WPILib\Scripts
