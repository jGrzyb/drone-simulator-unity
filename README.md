# drone-simulator-unity

Symulator fizyki i sterowania dronem w środowisku Unity umożliwiający trening latania oraz badanie zachowania drona dla różnych parametrów symulacji. 

### Model fizyki
Model fizyki składa się z następujących elementów:
- grawitacja
- siła ciągu śmigieł
- moment reakcyjny generowany przez śmigła
- ground effect

### Model sterowania
Model sterowania składa się z następujących elementów:
- regulator PD
- macierz alokacji mocy
- symulacja sensorów
- filtr Kalmana do estymacji pochylenia

### Funkcjonalności
Projekt posiada następujące funkcjonalności:
- Lot dronem
- Ustawianie parametrów drona
- Wybór poziomu realizmu symulacji
- Wybór mapy
- Lot rojem dronów

## Instrukcja kompilacji i uruchomienia
### Wymagania systemowe
- Środowisko: Unity Hub oraz Unity w wersji 6000.2.7f2

### Procedura kompilacji
- **Otwarcie projektu:** Otwórz projekt w edytorze Unity.
- **Konfiguracja kompilacji:** Wybierz z menu **File -> Build Settings**
- **Wybór platformy:** Wybierz docelowy system operacyjny
- **Kompilacja:** Naciśnij przycisk **Build**


### Uruchomienie projektu
#### Uruchomienie zbudowanego projektu
- Otwórz folder wskazany w procesie kompilacji
- Uruchom plik wykonywalny

#### Uruchomienie projektu w edytorze Unity
- Otwórz projekt w edytorze Unity
- Naciśnij przycisk **Play**