Robot sortujący koraliki według koloru:

1. Opis modelu:
Robot posiada pojemnik u góry, do którego wsypuje się koraliki różnego koloru. Mechanizm pobiera pojedynczy koralik, rozpoznaje jego kolor za pomocą czujnika, a następnie ruchem obrotowym umieszcza go w odpowiednim pojemniku.

2. Elementy konstrukcyjne:
1.	Pojemnik wsypowy:
o	Lejek z wąskim otworem na dnie.
o	Mechanizm dozujący (wirujący talerz dziurką na koralik), który wydaje pojedyncze koraliki.
2.	Czujnik koloru:
o	Zamontowany bezpośrednio pod mechanizmem dozującym.
o	Analizuje kolor koralika przed jego przekazaniem do sortowania.
3.	Mechanizm obrotowy:
o	Platforma z pojemnikami ułożonymi promieniście.
o	Obracana serwomechanizmem, który po wykryciu koloru kieruje koralik do odpowiedniego pojemnika.
4.	Pojemniki na posortowane koraliki:
o	Kilka przegród, na okrągłej podstawie, każda przypisana do innego koloru.





3. Proces sortowania:
1.	Wsypanie koralików:
o	Koraliki są wsypywane do pojemnika u góry robota.
2.	Dozowanie:
o	Mechanizm podaje pojedynczy koralik do czujnika.
3.	Analiza koloru:
o	Czujnik odczytuje dane RGB i klasyfikuje kolor.
4.	Ruch obrotowy:
o	Rurka obraca się w określone miejsce przypisane do danego koloru.
5.	Wrzut do pojemnika:
o	Koralik spada do odpowiedniego pojemnika.

4. Materiały i komponenty:
1.	Mikrokontroler: STM32F767ZI.
2.	Czujnik koloru: TCS34725.
3.	Serwomechanizm: 2x MG996R
4.	Silnik krokowy: NEMA17 17HS4023 42mm
5.	Mechanizm dozujący:
o	Drukowany 3D lub wykonany z plexi/metalowych części.
6.	Konstrukcja:
o	Sklejka, plexi lub elementy wydrukowane na drukarce 3D.
7.	Opcjonalnie: Wyświetlacz OLED/TFT do monitorowania wyników pracy.
