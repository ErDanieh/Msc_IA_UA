PY=python3
PIP=pip

all: engine gui

gui:
	cd connect6GUI && \
	$(PY) ConnectMore.py

engine: clean
	$(PY) -m PyInstaller __main__.py

install:
	$(PIP) install -r requirements.txt

# test:
# 	$(PY) -m unittest tests.TestNegamax

clean:
	rm -rf build dist __main__.spec tia-engine.log