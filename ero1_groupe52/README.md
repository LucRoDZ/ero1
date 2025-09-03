# ERO1 Groupe52

On s’intéresse à l’optimisation des trajets des équipes de déneigement de la ville de Montréal.

## Installation

installez python3 

```bash
sudo apt install python3
```

Si vous voulez executer les programmes un à un pour tester avec différentes valeurs sans passer par le script faites :

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

## Usage

En passant par le script pour avoir une démonstration complète :

```bash
./launch.sh
```
Vous obtiendrez ensuite deux fichiers html que vous pourrez ouvrir dans votre navigateur pour visualiser les différents parcours.

##

Pour tester le programme concernant les drones :

```bash
python3 src/display.py #--drones=1 --centers=5 --radius=500
```

drones : nombre de drone parcourant Montréal.

centers : nombre de zones enneigées et placées aléatoirement dans Montréal.

radius : rayon d'enneigement en mètre autour des centres placés.

