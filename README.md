# DemoPhysicsEngine
A 2023-as Berzsenyis fizikatáborban bemutatott szimulációs technikákat példázó demo program.

A program fehasználja a glm könyvtárat, ezt azonban nem másoltam be ide, helyette [innen](https://github.com/g-truc/glm) töltendő le.
A glm hasznos vektor és mátrix osztályokat és műveleteket valósít meg. Header only, szóval csak be kell másolni és működni fog.

## Hogyan használd

### World
Ezzel lehet létrehozni egy szimulációs egységet (világot), amely testeket tartalmaz, amik a szimuláció léptetésére mozognak, ütköznek egymással.\
\
```Vec2 gravity```\
A nehézségi gyorsulás vektora.\
\
```BodyRef createBody()```\
Új testet hoz létre a szimulációban. A BodyRef pointer tipusú, a heapen foglalt memóriára mutat, de ennek a felszabadítását NEM a user végzi.\
\
```void destroyBody(BodyRef body)```\
Törli a testet a szimulációból, felszabadítja a memóriát.\
\
```void update(float delta)```\
Delta idővel lépteti a szimulációt.

### Body
Egy merev testet ábrázoló osztály. A tagváltozóiban állítható a pozíciója, sebessége.\
\
```float friction```\
A súrlódási együtthatót nem objektum-objektum párokra adjuk meg, helyette objektumonként, és az értékeket átlagoljuk ütköyéskor. Ez nem túl valóságos, de elég jó közelítés.\
\
```BodyType type```
* DYNAMIC - A test mozoghat, normálisan vesz részt a szimulációban.
* STATIC - Elmozdíthatatlan test, végtelen tömeggel. Padló, falak létrehozásánál hasznos.

```Unique<Shape> shape```\
A test alakját meghatározó tagváltozó. A Shape egy abstract osztály, eddig egyetlen implementációja a Polygon, ami egy sokszöget ábrázol. Egy Polygon objektum points tagváltozója az alakzat pontjait tárolja a körüljárás sorrendjében.\
\
```template<class DataType, typename ... Args> void createUserData(Args ... args)```\
A testhez csatol egy DataType tipusú objektumot, amit úgy hoz létre, hogy az args... argumentumokkal meghícja a DataType ctor-át.\
\
```template<class DataType> DataType& getUserData()```\
Visszaadja a testhez csatolt objektumot DataType tipusúra cast-olva.
