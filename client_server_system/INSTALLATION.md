## OpenGL Setup in Windows

* Download the GLUT files (`glutdlls37beta.zip`) [here](https://www.opengl.org/resources/libraries/glut/glut_downloads.php)
* Extract the header file glut.h and paste it in: `C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\include\gl`
* Extract glut.lib and glut32.lib and paste them in: `C:\Program Files (x86)\Microsoft Visual Studio 12.0\VC\lib`
* Extract and put glut32.dll into: `C:\Windows\system32` (32-bit system), and glut.dll and glut32.dll into: `‪C:\Windows\SysWOW64` (64-bit system)