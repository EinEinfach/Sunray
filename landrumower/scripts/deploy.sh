#!/bin/bash

# Aktiviere die Python-Umgebung
# Pfad zur virtuellen Umgebung
VENV="$HOME/pico/.venv"
if [[ ! -f "$VENV/bin/activate" ]]; then
  echo "❌ Python-Umgebung nicht gefunden unter $VENV"
  exit 1
fi
source "$VENV/bin/activate"

# Aktuelle Datei (Argument 1) holen
FILE="$1"

if [[ -z "$FILE" ]]; then
  echo "❌ Keine Datei angegeben!"
  exit 1
fi

if [[ "$FILE" != *.py ]]; then
  echo "❌ Nur Python-Dateien erlaubt!"
  exit 1
fi

# Datei auf den Pico kopieren
echo "📤 Kopiere $FILE nach Pico..."
mpremote connect /dev/ttyACM1 cp "$FILE" :main.py

# Softreset
echo "🔁 Softreset..."
mpremote connect /dev/ttyACM1 reset

# Verbindung schliessen
echo "🔌 Verbindung wird sauber geschlossen..."
mpremote connect $PORT run 'pass'

# Deaktivere die Python-Umgebung
deactivate
