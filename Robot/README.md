# Robot operációs rendszerek és fejlesztői ökoszisztémák

## Gyakorlat menete

A gyakorlat során két fő témakörrel foglalkozunk:

1. [ros2_control](https://github.com/ros-controls/ros2_control) kontroller fejlesztése hattengelyes robothoz
2. [MoveIt 2](https://github.com/moveit/moveit2) keretrendszer alapvető funkcióinak megismerése

Célunk továbbá, hogy mindenkinél legyen egy működőképes ROS 2 fejlesztői környezet.

## Fejlesztői környezet előkészítése

A szükséges környezetet kétféleképpen biztosítottuk:

* **Virtuális gép** formájában
* **Dev Container** formájában

Mindkét megoldás megkönnyíti a munkát, és akár a házi feladathoz is használható.

### Virtuális gép

A virtuális gép OVA fájlját [ezen a linken](https://drive.google.com/drive/folders/1QVG72PD-RhH4-ImP4yRvStO2G1q4QJGc?usp=sharing) találjátok. Ez megnyitható például **VMware Workstation** vagy **VirtualBox** segítségével.

**VMware Workstation esetén:**

* Válaszd ki az **Open Virtual Machine** opciót
* Tallózd be az OVA fájlt

**VirtualBox esetén:**

* Nyisd meg a **File** menüt, majd válaszd ki az **Import Appliance** lehetőséget
* Tallózd be az OVA fájlt

A virtuális gépbe való belépéshez használd a *student* jelszót.

### Dev Container használata

A **Dev Container** technológiáról részletesebben a [Visual Studio Code weboldalán](https://code.visualstudio.com/docs/devcontainers/containers) olvashattok.

A konténer indításához szükséges szoftverek:

* [Visual Studio Code](https://code.visualstudio.com/) + [Dev Containers](https://marketplace.visualstudio.com/items?itemName=ms-vscode-remote.remote-containers) kiegészítő
* [Docker Desktop](https://www.docker.com/products/docker-desktop) &ndash; valójában a [Docker Engine](https://docs.docker.com/engine/install/) is elegendő lenne, de a Dev Containerekről szóló dokumentáció is a Desktop verziót ajánlja

Ha ezek mind telepítve vannak, akkor leklónuzhatjuk az `education` repository-t a saját gépünkre. Parancssorból például:

```bash
git clone https://github.com/kroshu/education.git -b viiiav55-starter
```

Ezután indítsuk el a [Docker daemont](https://docs.docker.com/engine/daemon/start/), ezt könnyen megtehetjük a Docker Desktop grafikus felületén keresztül is. Nyissuk meg az `education` repóban található `VIIIAV55` nevű mappát VS Code-ban. Nyomjuk meg `F1` gombot a `Command Palette` megnyitásához, majd válasszuk ki a `Dev Containers: Rebuild and Reopen in Container` opciót. Most már csak várnunk kell, hogy felépüljön a rendszer.

#### Eltérések céges hálózat esetén

Amennyiben a **KUKA hálózatáról** szeretnénk a környezetet összeállítani, akkor először meg kell szereznünk a KUKA gyökértanusítványát (*root certificate*), amit a `.devcontainer/certs` mappába kell elhelyeznünk. Ezen kívül ki kell kommentezni a `.devcontainer/devcontainer.json` fájlban a `mounts` kulcs utáni részt. Ezt követően minden ugyanúgy történik, mint a korábban leírt esetben.

#### Megjegyzés

A fentebb leírt lépéseket [Podman Desktop](https://podman-desktop.io/) mellett próbáltuk ki, amely a Docker Desktop egy nyílt forráskódú alternatívája. Podman használatához szükséges beállításokról a [VS Code dokumentációjában](https://code.visualstudio.com/remote/advancedcontainers/docker-options#_podman) olvashattok.

## Elvégzendő feladatok

### 1. Robot vezérlése saját kontrollerrel

Ebben a feladatban egy olyan kontrollert kell készíteni, amely egy ROS topicon keresztül kapott csuklópozíciókat követve mozgatja a robotot. A kontrollernek képesnek kell lennie új célpozíció fogadására akkor is, ha az előző célt még nem érte el. További követelmény, hogy a kontroller legyen paraméterezhető egy konfigurációs fájlon keresztül. A kiinduló ROS csomagot a `my_controller` mappában találjátok.

### 2. Robot mozgatása MoveIt segítségével

A második részben két példán keresztül ismerkedünk meg a MoveIt alapjaival:

1. Egy objektumot helyezünk el a virtuális térben, amit a robotnak ki kell kerülnie.
2. Megvalósítunk egy egyszerű pick & place alkalmazást.

A kiinduláshoz szükséges kódot a `moveit_example` mappában találjátok. Először a `first_scenario.hpp` fájlt egészítjük ki, ahol egy téglatestet helyezünk el a világban, majd a robotot a túloldalára mozgatjuk. Ezután a `second_scenario.hpp` fájlban egy előre elkészített környezetben kell a lila hengert egyik asztalról a másikra áthelyezni úgy, hogy az soha ne forduljon fejjel lefelé.

## Linkek

* [Tantárgyi adatlap](https://portal.vik.bme.hu/kepzes/targyak/VIIIAV55/)
* [ROS 2](https://github.com/ros2)
* [ros2_control](https://github.com/ros-controls/ros2_control)
* [MoveIt 2](https://github.com/moveit/moveit2)
* [Sajtómegjelenés](https://www.linkedin.com/posts/kukaglobal_a-kuka-%C3%A9s-a-bme-budapest-university-of-activity-7229396384574013440-LHqc/)

