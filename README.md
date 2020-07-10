# Video-Stabilization-Algorithm
B. Sc. Eng. Thesis in the field of Information Technology. Design and implementation of Video stabilization algorithm in Qt C++. Development of my own algorithm to stabilize videos. Its purpose is to remove all irrelevant high frequency vibrations from whole video sequence.


Zawartość repozytorium:
1. Plik pdf - artykuł dotyczący pracy inżynierskiej
2. Folder MyVideoStabilizer zawierający kod źródłowy aplikacji do stabilizowania filmów
3. Folder TwoSimultaneousVideosPlayer zawierający kod źródłowy aplikacji do porównywania filmu ustabilizowanego z oryginalnym lub ogólnie do porównywania 2 filmów
4. Plik output zawierający przykładowy rezultat działania aplikacji do stabilizowania filmów (rezultatem działania aplikacji jest też ustabilizowany film)

Przykładowe filmy do stabilizacji pobierałem ze strony
http://liushuaicheng.org/SIGGRAPH2013/database.html

Instrukcja kompilacji i instalacji aplikacji

1. Aplikacja została stworzona z użyciem biblioteki Qt, więc trzeba pobrać i zainstalować biblioteki Qt z internetu np. poprzez instalację środowiską Qt stąd https://www.qt.io/download

2. Aplikacja została stworzona z użyciem biblioteki OpenCV, więc trzeba pobrać i zainstalować biblioteki OpenCV z internetu
Instukcja przykładowej instalacji: https://www.pyimagesearch.com/2018/05/28/ubuntu-18-04-how-to-install-opencv/

3. Trzeba upewnić się że w 2 plikach MyVideoStabilizer.pro, TwoSimultaneousVideosPlayer.pro 
zmienna OPENCV_LIB_PATH wskazuje na folder ze skompilowanymi bibliotekami opencv .so, .a oraz 
zmienna OPENCV_INCLUDE_PATH wskazuje na folder z plikami nagłówkowymi .h, .hpp do dołączenia

U mnie to wygląda tak:
OPENCV_LIB_PATH=/home/pawel/opencv/build/lib
OPENCV_INCLUDE_PATH=/usr/local/include/opencv
u Państwa może być inaczej i może być konieczne odpowiednie ustawienie tych zmiennych, żeby wskazywały na właściwe foldery

4. Kompilacja kodu źródłowego

4.1 Sposób 1 - kompilacja z użyciem zintegrowanego środowiska programistycznego (IDE) z Qt
4.1.1 Trzeba otworzyć projekt wewnątrz środowiska Qt poprzez otwarcie projektu MyVideoStabilizer i wskazać na plik MyVideoStabilizer.pro
4.1.2 Potem trzeba nacisnąć młotek w lewym dolnym rogu dow celu skompilowania projektu

4.2 Sposób 2 - kompilacja ręczna w konsoli bez używania zintegrowanego środowiska programistycznego (IDE) z Qt
4.2.1 Trzeba przejść w konsoli bash lub cmd do folderu MyVideoStabilizer, w którym jest zawarty plik MyVideoStabilizer.pro
4.2.3 Następnie trzeba wywołać polecenie qmake, które generuje automatycznie plik Makefile (w windowsie jeśli nie została ustawiona zmienna środowiskowa QMAKE konieczne jest wpisanie całej ścieżki do programu qmake, typu C:\Qt\5.10.0\mingw53_32\bin\qmake.exe)
4.2.4 Potem trzeba wywołać polecenie make, które generuje automatycznie działającą aplikację (w windowsie jeśli nie została ustawiona zmienna środowiskowa MAKE konieczne jest wpisanie całej ścieżki do programu make, typu C:\Qt\Tools\mingw530_32\bin\mingw32-make.exe)

5. Uruchamianie aplikacji

5.1 Sposób 1 - uruchamianie z użyciem zintegrowanego środowiska programistycznego (IDE) z Qt
5.1.1 Trzeba przenieść katalog z przykładowymi filmami do folderu ze skompilowanym kodem binarnym aplikacji o nazwie typu "build-MyVideoStabilizer-Desktop_Qt_5_13_1_GCC_64bit-Debug"
5.1.2 Następnie trzeba nacisnąć zieloną strzałkę w lewym dolnym rogu w celu uruchomienia projektu

5.2 Sposób 2 - uruchamianie ręczne w konsoli bez używania zintegrowanego środowiska programistycznego (IDE) z Qt
5.2.1 Trzeba przenieść katalog z przykładowymi filmami do folderu z aplikacją "MyVideoStabilizer"
5.2.2 Następnie można już uruchomić aplikację z pliku MyVideoStabilizer.exe, np. poprzez polecenie ./MyVideoStabilizer (w windowsie może być konieczne ustawienie ścieżki do bibliotek Qt w zmiennych środowiskowych lub przekopiowanie bibliotek Qt do folderu w którym znajduje się plik exe, biblioteki Qt znajdują się w ścieżce typu C:\Qt\Tools\QtCreator\bin)

Przykładowe filmy do stabilizacji można pobrać ze strony
http://liushuaicheng.org/SIGGRAPH2013/database.html

