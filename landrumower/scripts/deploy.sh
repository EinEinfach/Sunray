#!/bin/bash

# === Konfiguration ===
VENV="$HOME/pico/.venv"
PORT="/dev/ttyACM1"
MAIN_FILE="main.py"
LIB_DIR="lib"

# === Virtualenv prüfen und aktivieren ===
if [[ ! -f "$VENV/bin/activate" ]]; then
  echo "❌ Python-Umgebung nicht gefunden unter $VENV"
  exit 1
fi
source "$VENV/bin/activate"

# === Datei prüfen ===
if [[ ! -f "$MAIN_FILE" ]]; then
  echo "❌ Datei $MAIN_FILE nicht gefunden!"
  deactivate
  exit 1
fi

# === "stop"-Befehl an Pico senden (wenn erwartet) ===
echo "🛑 Sende 'stop' an $PORT..."
echo "stop" > "$PORT"
sleep 1

# === main.py hochladen ===
echo "📤 Kopiere $MAIN_FILE..."
mpremote connect $PORT cp "$MAIN_FILE" :main.py || {
  echo "❌ Fehler beim Hochladen von $MAIN_FILE"
  deactivate
  exit 1
}

# === lib/ rekursiv hochladen ===
if [[ -d "$LIB_DIR" ]]; then
  echo "📂 Lade Inhalt von $LIB_DIR/ hoch..."

  # Alle Verzeichnisse erstellen
  find "$LIB_DIR" -type d | while read dir; do
    remote_dir=":${dir}"
    echo "📁 Erstelle Ordner $remote_dir auf dem Pico"
    mpremote connect $PORT mkdir "$remote_dir"
  done

  # Alle Dateien kopieren
  find "$LIB_DIR" -type f | while read file; do
    echo "📄 Kopiere Datei $file"
    mpremote connect $PORT cp "$file" ":$file"
  done
else
  echo "⚠️  Kein $LIB_DIR/-Ordner gefunden – wird übersprungen"
fi

# === Softreset ===
echo "🔁 Softreset..."
mpremote connect $PORT reset

# === Verbindung "beenden" ===
echo "🔌 Verbindung beenden..."
mpremote connect $PORT run 'pass'

# === Deaktivieren der virtuellen Umgebung ===
deactivate
echo "✅ Deployment abgeschlossen."
