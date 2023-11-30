from time import *;
import os;
import random;
from subprocess import *;
from threading import *;

class Move:
    NONE = 0;
    BLACK = 1;
    WHITE = 2;
    EDGE = 19;
    def __init__(self, color = NONE, x1 = -1, y1 = -1, x2 = -1, y2 = -1):
        self.color = color;
        self.x1 = x1;
        self.y1 = y1;
        self.x2 = x2;
        self.y2 = y2;

    def __str__(self):
        return 'color: {0}, x1: {1}, y1: {2}, x2: {3}, y2: {4}'.format(self.color, self.x1, self.y1, self.x2, self.y2);

    def fromCmd(cmd, color = None):
        # print(cmd);
        # print(self);
        cmd = cmd.strip();
        if cmd.startswith('move '):
            cmd = cmd[5:].upper();
            if len(cmd) == 2:
                cmd = cmd*2;
            m = Move(color);
            m.x1 = ord(cmd[0]) - ord('A');
            m.y1 = ord(cmd[1]) - ord('A');
            m.x2 = ord(cmd[2]) - ord('A');
            m.y2 = ord(cmd[3]) - ord('A');
            return m;
        
        return None;

    def toCmd(self):  
        cmd = 'move ' + self.cmd() + '\n';
        print('Cmd:', cmd);
        return cmd;

    def toPlaceCmd(self):
        if self.color == Move.BLACK:
            cmd = 'black ';
        elif self.color == Move.WHITE:
            cmd = 'white ';
        else:
            return 'None Place Cmd\n';
        cmd += self.cmd() + '\n';
        # print('Cmd:', cmd);
        return cmd;

    def cmd(self):
        base = ord('A');
        return chr(base + self.x1) + chr(base + self.y1) + chr(base + self.x2) + chr(base + self.y2);

    def invalidate(self):
        self.color= None;
        self.x1 = -1;
        self.y1 = -1;
        self.x2 = -1;
        self.y2 = -1;

    def isValidated(self):
        if self.color != Move.BLACK and self.color != Move.WHITE:
            return False;
        if Move.isValidPosition(self.x1, self.y1) and Move.isValidPosition(self.x2, self.y2):
            return True;

        return False;

    def isValidPosition(x, y):
        if x >= 0 and x < Move.EDGE and y >= 0 and y < Move.EDGE:
            return True;
        return False;

class GameEngine:
    def __init__(self):
        #self.fileName = GameEngine.getDefaultEngineFile();
        self.fileName = None
        self.proc = None;
        self.move = Move();
        self.color = Move.NONE;
        self.setName('Unknown');

        '''def getDefaultEngineFile():
        # Check the os, supported Linux/Mac/Windows.
        defaultEngineFile = '';
        if os.name == 'nt':
            defaultEngineFile = 'engines/cloudict.exe';
        else:
            osName = os.uname()[0];
            if osName == 'Darwin':
                defaultEngineFile = 'engines/cloudict.app';
            elif osName == 'Linux':
                defaultEngineFile = 'engines/cloudict.linux';
            else:
                print('Not supported OS');
                exit(-1);
        return defaultEngineFile;'''
        
    def init(self, fileName = None, depth = None, vcf = None):
        self.release();

        if fileName != None and fileName.strip() != '':
            self.fileName = fileName;
        else:
            fileName = self.fileName;
        #print('init:', self.fileName);
        if os.name == 'nt':
            # Windows NT hide
            startupinfo =  STARTUPINFO();
            startupinfo.dwFlags |= STARTF_USESHOWWINDOW;
            self.proc = Popen(fileName, stdin=PIPE, stdout=PIPE, bufsize=0, startupinfo=startupinfo);
        else:
            self.proc = Popen(fileName, stdin=PIPE, stdout=PIPE, bufsize=0);

        # game engine name
        self.setName(fileName);
        self.sendCmd('name\n');
        while True:
            msg = self.waitForNextMsg();
            if msg.startswith('name '):
                self.setName(msg.split(' ')[1]);
                break;

        if depth != None:
            cmd = 'depth ' + str(depth) + '\n';
            # print(cmd);
            self.sendCmd(cmd);
        if vcf != None:
            if vcf:
                cmd = 'vcf\n';
            else:
                cmd = 'unvcf\n';
            # print(cmd);
            self.sendCmd(cmd);
            
        self.move.invalidate();

        return True;
        
    def isReady(self):
        return self.proc != None

    def setName(self, name):
        self.name = self.shortName = name;
        if len(self.shortName) > 10 and self.shortName.find('.') > -1:
            ls = self.shortName.split('.');
            for i in ls:
                if i != '':
                    self.shortName = i;
                    break;
        if len(self.shortName) > 10:
            self.shortName = self.shortName[:8] + '...';

    def release(self):
        while self.proc != None:
            if self.proc.poll() == None:
                self.proc.terminate();
                # self.sendCmd('quit\n');
                # print('Release');
                sleep(0.2);
            else:
                self.proc = None;
                break;
        self.move.invalidate();
        self.msg = None

    def next(self, moveList = []):
        if self.proc != None:
            cmd = 'new xxx\n';
            self.sendCmd(cmd);
            for m in moveList:
                cmd = m.toPlaceCmd();
                self.sendCmd(cmd);

            cmd = 'next\n';
            self.sendCmd(cmd);

    def sendCmd(self, cmd):
        if self.proc != None:
            try:
                # print('sendCmd to stdin:', cmd);
                if len(cmd) < 1 or cmd[-1] != '\n':
                    # Add ret in the end;
                    cmd += '\n';
                self.proc.stdin.write(cmd.encode());
            except Exception as e:
                print('Error for sendCmd:', cmd, str(e));

    def waitForNextMsg(self):
        if self.proc != None:
            try:
                # print('Waiting');
                self.msg = self.proc.stdout.readline().decode();
                # print('out:', self.msg);
            except Exception as e:
                print('Error for waitForNextMsg:', str(e));
        return self.msg;
        
