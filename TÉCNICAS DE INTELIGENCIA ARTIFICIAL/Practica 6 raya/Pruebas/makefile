PY=python3
PIP=pip

all: engine gui

gui:
	cd lib/connect6GUI && \
	$(PY) ConnectMore.py

engine: clean
	$(PY) -m PyInstaller main.py

install:
	$(PIP) install -r requirements.txt

clean:
	rm -rf build dist main.spec tia-engine.log