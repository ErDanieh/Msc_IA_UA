#
# Copyright (c) 2014, Liang Li <ll@lianglee.org; liliang010@gmail.com>,
# All rights reserved.
#
# This program is a free software; you can redistribute it and/or modify
# it under the terms of the BSD license. See LICENSE.txt for details.
#
# It's a UI for Coudict(https://github.com/lang010/cloudict) based on Python 3.
# Other connect6 programs with similar commands are also supported.
# Enjoy Connect6;)
#
# Last Modified: 2014/09/14
#

from tkinter import *;
from tkinter import filedialog;
from tkinter import messagebox;
from subprocess import *;
from threading import *;
from time import *;
import os;
import random;
from tournament import *
from engine import *
import time

if os.name == 'nt':
    from subprocess import STARTUPINFO;

class App(Frame):
    

    def __init__(self, master=None):
        Frame.__init__(self, master, width=640, height=700)
        self.pack();

        # Game state: -1 -> quit, 0 -> first, 1 -> second, 2 -> gameEngine 3;
        self.gameMode = GameState.Idle;
        self.gameState = GameState.Idle;
        #Bots
        self.botPlayerBlack = BotPlayer()
        self.botPlayerWhite = BotPlayer()
        
        #Predefined game
        self.predefGame = Game(HumanPlayer(), HumanPlayer())
        
        #Current game
        self.currentGame = self.predefGame
        #Time
        self.currentTime = time.process_time()
        
        #Tournament
        self.tournament = Tournament()

        self.initResource();

        self.createBoard();
        
        self.initBoard();

    def destroy(self):
        self.gameState = GameState.Exit;
        self.botPlayerBlack.release();
        self.botPlayerWhite.release();
        self.currentGame.release();
        self.searchThread.join();
        Frame.destroy(self);

    def initResource(self):

        # Images sets.
        self.images = {};
        im = self.images;
        im['go_u'] = PhotoImage(file='imgs/go_u.gif');
        im['go_ul'] = PhotoImage(file='imgs/go_ul.gif');
        im['go_ur'] = PhotoImage(file='imgs/go_ur.gif');
        im['go'] = PhotoImage(file='imgs/go.gif');
        im['go_l'] = PhotoImage(file='imgs/go_l.gif');
        im['go_r'] = PhotoImage(file='imgs/go_r.gif');
        im['go_d'] = PhotoImage(file='imgs/go_d.gif');
        im['go_dl'] = PhotoImage(file='imgs/go_dl.gif');
        im['go_dr'] = PhotoImage(file='imgs/go_dr.gif');
        im['go_-'] = PhotoImage(file='imgs/go_-.gif');
        im['go_b'] = PhotoImage(file='imgs/go_b.gif');
        im['go_w'] = PhotoImage(file='imgs/go_w.gif');
        im['go_bt'] = PhotoImage(file='imgs/go_bt.gif');
        im['go_wt'] = PhotoImage(file='imgs/go_wt.gif');

        im['angel'] = PhotoImage(file='imgs/Emotes-face-angel.gif');
        im['laugh'] = PhotoImage(file='imgs/Emotes-face-laugh.gif');
        im['plain'] = PhotoImage(file='imgs/Emotes-face-plain.gif');
        im['raspberry'] = PhotoImage(file='imgs/Emotes-face-raspberry.gif');
        im['sad'] = PhotoImage(file='imgs/Emotes-face-sad.gif');
        im['smile'] = PhotoImage(file='imgs/Emotes-face-smile.gif');
        im['smile-big'] = PhotoImage(file='imgs/Emotes-face-smile-big.gif');
        im['surprise'] = PhotoImage(file='imgs/Emotes-face-surprise.gif');
        im['uncertain'] = PhotoImage(file='imgs/Emotes-face-uncertain.gif');
        im['wink'] = PhotoImage(file='imgs/Emotes-face-wink.gif');

        self.faces = {};
        waiting = [im['angel'], im['raspberry'], im['smile'], im['wink']];
        self.faces[GameState.Idle] = waiting;
        self.faces[GameState.WaitForHumanFirst] = waiting;
        self.faces[GameState.WaitForHumanSecond] = waiting;
        waitingSad = [im['plain'], im['sad'], im['surprise'], im['uncertain'] ];
        self.faces['LowScore'] = waitingSad;
        searching = [im['plain'], im['surprise'], im['uncertain'] ];
        self.faces[GameState.WaitForEngine] = searching;
        won = [im['angel'], im['laugh'], im['raspberry'], im['smile'], im['smile-big'], im['wink'] ];
        self.faces['win'] = won;
        lost = [im['plain'], im['sad'], im['surprise'], im['uncertain'], ];
        self.faces['lose'] = lost;

        # Searching thread
        self.searchThread = Thread(target = self.searching);
        self.searchThread.start();
        self.showDisplayMsg = True

        # Widgets
        self.canvas = Canvas(self, width=640, height=640);
        self.canvas.pack(side=LEFT, fill=BOTH, expand=1);
        # Button widgets
        self.controlFrame = LabelFrame(self);
        self.controlFrame.pack(fill=BOTH, expand=1);
        
        self.controlFrame.aiLevel = labelframe = LabelFrame(self.controlFrame, text='AI Level');
        labelframe.pack(fill=X, expand=1);
        self.aiLevel = IntVar();
        #print(self.aiLevel.get());
        labelframe.lowRBtn = Radiobutton(labelframe, text="Low", variable=self.aiLevel, value=4);
        labelframe.lowRBtn.select();
        labelframe.lowRBtn.pack( anchor = W );
        labelframe.mediumRBtn = Radiobutton(labelframe, text="Medium", variable=self.aiLevel, value=5);
        labelframe.mediumRBtn.pack( anchor = W )
        labelframe.highRBtn = Radiobutton(labelframe, text="High", variable=self.aiLevel, value=6);
        labelframe.highRBtn.pack( anchor = W );
        self.vcf = IntVar();
        chbox = Checkbutton(labelframe, text = "With VCF", variable = self.vcf, );
        chbox.select();
        chbox.pack(anchor = W );
        # print(self.vcf.get());

        self.controlFrame.selectBlack = labelframe = LabelFrame(self.controlFrame, text='Black Player');
        labelframe.pack(fill=X, expand=1);
        labelframe.blackImg = Label(labelframe, image=self.images['go_b']);
        labelframe.blackImg.pack(side=LEFT, anchor = W);

        self.blackOption = IntVar();
        labelframe.humanRBtn = Radiobutton(labelframe, text="Human", value=0, variable=self.blackOption, command=self.setBlackHuman);
        labelframe.humanRBtn.select();
        labelframe.humanRBtn.pack( anchor = W );
        labelframe.engineRBtn = Radiobutton(labelframe, text="AI", value=1, variable=self.blackOption, command=self.setBlackBot);
        labelframe.engineRBtn.pack( anchor = W );
        
        self.controlFrame.selectWhite = labelframe = LabelFrame(self.controlFrame, text='White Player');
        labelframe.pack(fill=X, expand=1);
        labelframe.whiteImg = Label(labelframe, image=self.images['go_w']);
        labelframe.whiteImg.pack(side=LEFT, anchor = W);

        self.whiteOption = IntVar();
        labelframe.humanRBtn = Radiobutton(labelframe, text="Human", value=0, variable=self.whiteOption, command=self.setWhiteHuman);
        labelframe.humanRBtn.select();
        labelframe.humanRBtn.pack( anchor = W );
        labelframe.engineRBtn = Radiobutton(labelframe, text="AI", value=1, variable=self.whiteOption,command=self.setWhiteBot);
        labelframe.engineRBtn.pack( anchor = W );
        
        self.controlFrame.gameContral = labelframe = LabelFrame(self.controlFrame, text='Game Control');
        labelframe.pack(fill=X, expand=1);
        labelframe.newBtn = Button(labelframe, text='Start Game', command=self.newSingleGame);
        labelframe.newBtn.pack(side=TOP, fill=X);
        labelframe.backBtn = Button(labelframe, text='Back Move', command=self.backMove);
        labelframe.backBtn.pack(fill=X);
        #Load engines
        labelframe.loadBtn = Button(labelframe, text='Load Black Engine', command=self.loadGameEngineBlack);
        labelframe.loadBtn.pack(fill=BOTH);
        labelframe.loadBtn2 = Button(labelframe, text='Load White Engine', command=self.loadGameEngineWhite);
        labelframe.loadBtn2.pack(fill=BOTH);
        labelframe.quitBtn = Button(labelframe, text='Quit Game', command=self.master.destroy);
        labelframe.quitBtn.pack(fill=X);
        
        self.controlFrame.tournament = labelframe = LabelFrame(self.controlFrame, text='Tournament');
        labelframe.pack(fill=X, expand=1);
        labelframe.loadBtn = Button(labelframe, text='Load Tournament', command=self.loadTournament);
        labelframe.loadBtn.pack(side=TOP, fill=X);
        labelframe.newBtn = Button(labelframe, text='Start Games', command=self.startTournamentGames);
        labelframe.newBtn.pack(side=TOP, fill=X);
        labelframe.newBtn = Button(labelframe, text='Save results', command=self.saveTournamentGames);
        labelframe.newBtn.pack(side=TOP, fill=X);
        

        self.controlFrame.aiStatus = labelframe = LabelFrame(self.controlFrame, text='AI Status');
        labelframe.pack(side=BOTTOM, fill=BOTH, expand="yes");
        labelframe.nameBlack = Label(labelframe, text='AI B Name');
        labelframe.nameBlack.pack(side=TOP, anchor = W);
        labelframe.nameWhite = Label(labelframe, text='AI W Name');
        labelframe.nameWhite.pack(side=TOP, anchor = W);
        labelframe.image = Label(labelframe, image=self.images['smile']);
        labelframe.image.pack(side=TOP, anchor = W);
        labelframe.info = Label(labelframe, text='');
        labelframe.info.pack(side=BOTTOM, anchor = W);

        self.updateStatus();
        
    def newSingleGame(self):
        self.showDisplayMsg = True
        self.tournament = Tournament()
        self.newGame()
        
    def setBlackHuman(self):
        self.predefGame.black = HumanPlayer()

    def setBlackBot(self):
        self.predefGame.black = self.botPlayerBlack
        
    def setWhiteHuman(self):
        self.predefGame.white = HumanPlayer()
        
    def setWhiteBot(self):
        self.predefGame.white = self.botPlayerWhite

    def isVcf(self):
        vcf = True;
        if self.vcf.get() < 1:
            vcf = False;
        # print('VCF', vcf);
        return vcf;
        
    def loadTournament(self):
        path = filedialog.askopenfilename(title='Load tournament file ', initialdir='tournaments');
        print('Load tournament file:', path);
        if len(path) > 0:
            try:
                self.tournament = RoundRobinTournament()
                self.tournament.load_from_file(path)
                print('Tournament loaded with ' + str(len(self.tournament.players)) + ' players')
            except Exception as e:
                messagebox.showinfo("Error","Error to generate tournament from: " + path + ",\n errors: " + str(e));
                self.tournament = Tournament()
                
    def saveTournamentGames(self):
        f = filedialog.asksaveasfile(mode='w', defaultextension=".txt", title='Save tournament file ', initialdir='tournaments');
        if f is None: # asksaveasfile return `None` if dialog closed with "cancel".
            return
            
        print('Saving tournament results');
        try:
            self.tournament.save_results(f)
            f.close()
            print('Tournament results saved');
        except Exception as e:
            messagebox.showinfo("Error","Error to save tournament. \n errors: " + str(e));
    
    #Gnerate games from tournament            
    def startTournamentGames(self):
        self.showDisplayMsg = False
        self.tournament.generate_games()
        
        #Execute first game
        game = self.tournament.next_game()
        if game is None:
            return
                
        self.currentGame = game
        #Execute game
        self.newGame()
        

    def loadGameEngineBlack(self):
        self.botPlayerBlack.path = filedialog.askopenfilename(title='Load executable file for new game engine black ', initialdir='engines');
        print('Load game engine black:', self.botPlayerBlack.path);
        if self.botPlayerBlack.has_correct_name():
            try:
                self.initGameEngine(self.botPlayerBlack, Move.BLACK);
                self.botPlayerBlack.release();
            except Exception as e:
                messagebox.showinfo("Error","Error to load the engine: " + self.botPlayerBlack.path + ",\n errors: " + str(e));
                self.botPlayerBlack.path = None
                
    def loadGameEngineWhite(self):
        self.botPlayerWhite.path = filedialog.askopenfilename(title='Load executable file for new game engine white ', initialdir='engines');
        print('Load game engine white:', self.botPlayerWhite.path);
        if self.botPlayerWhite.has_correct_name():
            try:
                self.initGameEngine(self.botPlayerWhite, Move.WHITE);
                self.botPlayerWhite.release();
            except Exception as e:
                messagebox.showinfo("Error","Error to load the engine: " + self.botPlayerWhite.path + ",\n errors: " + str(e));
                self.botPlayerWhite.path = None

    def initGameEngine(self, bot, move):
        bot.init_engine(self.aiLevel.get(), self.isVcf(), move);
        # Change the engine name
        shortName = bot.get_short_name().capitalize();
        self.controlFrame.aiLevel['text'] = 'AI Level';
        
        name = bot.get_name().capitalize();
        
        if move == Move.BLACK:
            self.controlFrame.aiStatus.nameBlack['text'] = name;
            self.controlFrame.selectBlack.engineRBtn['text'] = shortName;
        elif move == Move.WHITE:
            self.controlFrame.aiStatus.nameWhite['text'] = name;
            self.controlFrame.selectWhite.engineRBtn['text'] = shortName;
        #root.title('Cloudict.Connect6 - ' + name);

    def createBoardUnit(self, x, y, imageKey):
        lb = Label(self.canvas, height=32, width=32);
        lb.x = x;
        lb.y = y;
        lb['image'] = self.images[imageKey];
        lb.initImage = self.images[imageKey];
        lb.bind('<Button-1>', self.onClickBoard);
        self.gameBoard[x][y] = lb;

        return lb;

    def createBoard(self):
        self.gameBoard = [ [ 0 for i in range(Move.EDGE) ] for i in range(Move.EDGE)];
        self.moveList = [];
        images = self.images;
        gameBoard = self.gameBoard;
        canvas = self.canvas;
        # Upper
        self.createBoardUnit(0, 0, 'go_ul');
        for j in range(1, 18):
            self.createBoardUnit(0, j, 'go_u');
        self.createBoardUnit(0, 18, 'go_ur');

        # Middle
        for i in range(1,18):
            gameBoard[i][0] = self.createBoardUnit(i, 0, 'go_l');
            for j in range(1,18):
                gameBoard[i][j] = self.createBoardUnit(i, j, 'go');

            gameBoard[i][18] = self.createBoardUnit(i, 18, 'go_r');

        # Block point in the board
        self.createBoardUnit(3, 3, 'go_-');
        self.createBoardUnit(3, 9, 'go_-');
        self.createBoardUnit(3, 15, 'go_-');
        self.createBoardUnit(9, 3, 'go_-');
        self.createBoardUnit(9, 9, 'go_-');
        self.createBoardUnit(9, 15, 'go_-');
        self.createBoardUnit(15, 3, 'go_-');
        self.createBoardUnit(15, 9, 'go_-');
        self.createBoardUnit(15, 15, 'go_-');

        # Down
        self.createBoardUnit(18, 0, 'go_dl');
        for j in range(1,18):
            self.createBoardUnit(18, j, 'go_d');
        self.createBoardUnit(18, 18, 'go_dr');

    def backMove(self):
        if self.gameMode == GameState.AI2Human or self.gameMode == GameState.Human2Human:
            if self.gameState == GameState.WaitForHumanFirst:
                if self.gameMode == GameState.AI2Human and len(self.moveList) > 1:
                    # Back to 2 Move
                    self.unmakeTopMove();
                    self.unmakeTopMove();
                elif self.gameMode == GameState.Human2Human and len(self.moveList) > 0:
                    # Back to 1 Move
                    self.unmakeTopMove();
            elif self.gameState == GameState.WaitForHumanSecond:
                self.unplaceColor(self.move.x1, self.move.y1);
                self.toGameState(GameState.WaitForHumanFirst);

    def initBoard(self):
        self.moveList = [];
        self.times = []
        for i in range(Move.EDGE):
            for j in range(Move.EDGE):
                self.unplaceColor(i, j);
        self.remainingMoves = 19*19

    def unplaceColor(self, i, j):
        gameBoard = self.gameBoard;
        gameBoard[i][j]['image'] = gameBoard[i][j].initImage;
        gameBoard[i][j].color = 0;
        gameBoard[i][j].grid(row=i, column=j);

    def connectedByDirection(self, x, y, dx, dy):
        gameBoard = self.gameBoard;
        cnt = 1;
        xx = dx; yy = dy;
        while Move.isValidPosition(x+xx, y+yy) and gameBoard[x][y].color == gameBoard[x+xx][y+yy].color:
            xx += dx; yy += dy;
            cnt += 1;
        xx = -dx; yy = -dy;
        while Move.isValidPosition(x+xx, y+yy) and gameBoard[x][y].color == gameBoard[x+xx][y+yy].color:
            xx -= dx; yy -= dy;
            cnt += 1;
        if cnt >= 6:
            return True;
        return False;
        
    def connectedBy(self, x, y):
        # Four direction
        if self.connectedByDirection(x, y, 1, 1):
            return True;
        if self.connectedByDirection(x, y, 1, -1):
            return True;
        if self.connectedByDirection(x, y, 1, 0):
            return True;
        if self.connectedByDirection(x, y, 0, 1):
            return True;
        return False;

    def isWin(self, move):
        if move.isValidated():
            return self.connectedBy(move.x1, move.y1) or self.connectedBy(move.x2, move.y2);
        return False;

    def nextColor(self):
        color = Move.BLACK;
        if len(self.moveList) % 2 == 1:
            color = Move.WHITE;
        return color;

    def waitForMove(self, currGameEngine):
        color = self.nextColor();
        while True:
            # print('waitForMove');
            msg = currGameEngine.waitForNextMsg();
            # print('Msg:', msg);
            move = Move.fromCmd(msg, color);
            # print('Wait move:', move);
            self.updateStatus();
            if move != None:
                break;
            
        return move

    #Threaded search
    def searching(self):
        while True:
            try:
                if self.gameState == GameState.Exit:
                    break;
                if(self.gameState != GameState.Win and self.gameState != GameState.Draw and self.gameState != GameState.Idle):
                    if self.gameMode == GameState.AI2AI or self.gameMode == GameState.AI2Human:
                        if self.gameState == GameState.WaitForEngine:
                            color = self.nextColor()
                            #print(len(self.moveList), color)
                            currEngine = None
                            if(color == Move.BLACK):
                                currEngine = self.currentGame.black.engine;
                            else:
                                currEngine = self.currentGame.white.engine;
                                
                            currEngine.next(self.moveList);
                            move = self.waitForMove(currEngine);
                            currEngine.color = move.color;
                                
                            if(self.gameState != GameState.Win and self.gameState != GameState.Draw and self.gameState != GameState.Idle):
                                self.makeMove(move);
                                if self.gameState == GameState.WaitForEngine and self.gameMode == GameState.AI2Human:
                                    self.toGameState(GameState.WaitForHumanFirst);
                                    
                            sleep(0.05)
                            
                        else:
                            sleep(0.1);
                    else:
                        sleep(0.2);
                else:
                    sleep(0.1)
            except Exception as e:
                print('Exception when searching: ' + str(e));
                sleep(0.5);

    def updateStatus(self):
        image = random.sample(self.faces.get(GameState.Idle), 1)[0];
        ls = self.faces.get(self.gameState);
        # According to gameState.
        if ls != None and len(ls) > 0:
            image = random.sample(ls, 1)[0];
            
        self.controlFrame.aiStatus.image['image'] = image;
        self.controlFrame.aiStatus.info['text'] = '';

        msg = 'Press start to game.';
        
        #Game finished
        if self.gameState == GameState.Win or self.gameState == GameState.Draw:
            if self.winner == Move.BLACK:
                msg = 'Black Wins!';
            elif self.winner == Move.WHITE:
                msg = 'White Wins!';
            else:
                msg = "Draw!"
            
            #Copy moves and result
            self.currentGame.moves = self.moveList
            self.currentGame.times = self.times
            self.currentGame.result = self.winner
            
            nextGame = self.tournament.next_game()
                            
            if nextGame is None:
                self.currentGame = self.predefGame
            else:
                self.currentGame = nextGame
                sleep(1.0)
                self.newGame()
            
        elif self.gameState == GameState.WaitForHumanFirst:
            msg = 'Move the first...';
        elif self.gameState == GameState.WaitForHumanSecond:
            msg = 'Move the second...';
        elif self.gameState == GameState.WaitForEngine:
            # Check format: Searching 31/37
            currentEngine = None
            if self.nextColor() == Move.BLACK:
                currentEngine = self.currentGame.black.engine;
            else:
                currentEngine = self.currentGame.white.engine;
                
            msg = currentEngine.name+' Thinking.';
            
            if currentEngine.msg.startswith('Searching '):
                s = currentEngine.msg.split(' ')[1];
                ls = s.split('/');
                cnt = float(ls[0])/float(ls[1]) * 15;
                msg += '.' * int(cnt);
        self.controlFrame.aiStatus.info['text'] = msg;

            
    def otherColor(self, color):
        if color == Move.BLACK:
            return Move.WHITE;
        elif color == Move.WHITE:
            return Move.BLACK;
        return Move.NONE;
        
    def setBotNames(self):
        black = self.currentGame.black
        white = self.currentGame.white
               
        if black.type == Player.BOT:
            self.controlFrame.aiStatus.nameBlack['text'] = black.get_short_name().capitalize().strip();
            self.currentGame.black.name = black.get_name().capitalize().strip();
            #self.controlFrame.selectBlack.engineRBtn['text'] = black.get_name().capitalize();
        if white.type == Player.BOT:
            self.controlFrame.aiStatus.nameWhite['text'] = white.get_short_name().capitalize().strip();
            self.currentGame.white.name = white.get_name().capitalize().strip();
            #self.controlFrame.selectWhite.engineRBtn['text'] = white.get_name().capitalize();

    def newGame(self):
        #Release engines
        self.botPlayerBlack.release();
        self.botPlayerWhite.release();
        self.currentGame.release();
        
        self.initBoard();
        
        b_ready, w_ready = self.currentGame.is_ready()
        if(not b_ready):
            messagebox.showinfo("Error","Black engine is not ready");
            return
        elif (not w_ready):
            messagebox.showinfo("Error","White engine is not ready");
            return
            
        #Prepare players
        self.currentGame.start_players(self.aiLevel.get(), self.isVcf())
        self.setBotNames()
        
        mode, next_state = self.currentGame.get_game_state()
        self.currentTime = time.process_time()
        self.toGameMode(mode)
        self.toGameState(next_state)

    def addToMoveList(self, move):
        # Rerender pre move
        n = len(self.moveList);
        if n > 0:
            m = self.moveList[n-1];
            self.placeColor(m.color, m.x1, m.y1);
            self.placeColor(m.color, m.x2, m.y2);
            
        self.moveList.append(move);

    def unmakeTopMove(self):
        if len(self.moveList) > 0:
            m = self.moveList[-1];
            self.moveList = self.moveList[:-1];
            self.unplaceColor(m.x1, m.y1);
            self.unplaceColor(m.x2, m.y2);
            if len(self.moveList) > 0:
                m = self.moveList[-1];
                self.placeColor(m.color, m.x1, m.y1, 't');
                self.placeColor(m.color, m.x2, m.y2, 't');

    def makeMove(self, move):
        if move.isValidated():
            if(self.gameState != GameState.Win and self.gameState != GameState.Draw):
                #Calculate time spent
                t_end = time.process_time()
                t_delayed = t_end - self.currentTime
                self.times.append(t_delayed)
                
                self.addToMoveList(move);
                nextValidMove = self.placeStone(move.color, move.x1, move.y1);
                if(nextValidMove):
                    self.placeStone(move.color, move.x2, move.y2);
                else:
                    move.x2 = move.x1
                    move.y2 = move.y1
                    
        self.currentTime = time.process_time()
            # print('Made move:', move);
        return move;

    def placeStone(self, color, x, y):
        #Check illegal move
        if not self.isNoneStone(x, y):
            return True
    
        self.placeColor(color, x, y, 't');
        
        if self.connectedBy(x, y):
            self.winner = color;
            self.toGameState(GameState.Win);
            if color == Move.BLACK:
                if self.showDisplayMsg:
                    messagebox.showinfo("Black Win", "Black Win ;) Impressive!")
            else:
                if self.showDisplayMsg:
                    messagebox.showinfo("White Win", "White Win ;) Impressive!")
            return False
             
        self.remainingMoves = self.remainingMoves-1;   
        if self.remainingMoves == 0:
            self.winner = Move.NONE;
            self.toGameState(GameState.Draw);
            if self.showDisplayMsg:
                messagebox.showinfo("Draw", "Draw ;) Impressive!")
            return False
            
        return True

    def placeColor(self, color, x, y, extra = ''):
        if color == Move.BLACK:
            imageKey = 'go_b';
        elif color == Move.WHITE:
            imageKey = 'go_w';
        else:
            return ;
        imageKey += extra;
        self.gameBoard[x][y].color = color;
        self.gameBoard[x][y]['image'] = self.images[imageKey];
        self.gameBoard[x][y].grid(row=x, column=y);

    def isNoneStone(self, x, y):
        return self.gameBoard[x][y].color == Move.NONE;

    def toGameMode(self, mode):
        self.gameMode = mode;

    def toGameState(self, state):
        self.gameState = state;
        self.updateStatus();

    def onClickBoard(self, event):
        x = event.widget.x;
        y = event.widget.y;
        if not self.isNoneStone(x, y):
            return ;
        if self.gameMode == GameState.Human2Human:
            color = self.nextColor();
            if len(self.moveList) == 0:
                # First Move for Black
                self.move = Move(color, x, y, x, y);
                self.placeStone(self.move.color, x, y);
                self.addToMoveList(self.move);
                if(self.gameState != GameState.Win and self.gameState != GameState.Draw):
                    self.toGameState(GameState.WaitForHumanFirst);
                
            elif self.gameState == GameState.WaitForHumanFirst:
                self.move = Move(color, x, y);
                self.placeStone(self.move.color, x, y);
                if(self.gameState != GameState.Win and self.gameState != GameState.Draw):
                    if self.gameState == GameState.WaitForHumanFirst:
                        self.toGameState(GameState.WaitForHumanSecond);
                
            elif self.gameState == GameState.WaitForHumanSecond:
                self.move.x2 = x;
                self.move.y2 = y;
                self.placeStone(self.move.color, x, y);
                self.addToMoveList(self.move);
                if(self.gameState != GameState.Win and self.gameState != GameState.Draw):
                    if self.gameState == GameState.WaitForHumanSecond:
                        self.toGameState(GameState.WaitForHumanFirst);
            
            return ;
        
        if self.gameMode == GameState.AI2Human:
            color = self.nextColor();
            # print(color);
            if len(self.moveList) == 0 and self.gameState == GameState.WaitForHumanFirst:
                # First Move for Black
                self.move = Move(color, x, y, x, y);
                self.addToMoveList(self.move);
                self.placeStone(self.move.color, x, y);
                if(self.gameState != GameState.Win and self.gameState != GameState.Draw):
                    self.toGameState(GameState.WaitForEngine);
                
            elif self.gameState == GameState.WaitForHumanFirst:
                self.move = Move(color, x, y);
                self.placeStone(self.move.color, x, y);
                if(self.gameState != GameState.Win and self.gameState != GameState.Draw):
                    if self.gameState == GameState.WaitForHumanFirst:
                        self.toGameState(GameState.WaitForHumanSecond);
                    
            elif self.gameState == GameState.WaitForHumanSecond:
                self.move.x2 = x;
                self.move.y2 = y;
                self.placeStone(self.move.color, x, y);
                self.addToMoveList(self.move);
                if(self.gameState != GameState.Win and self.gameState != GameState.Draw):
                    if self.gameState == GameState.WaitForHumanSecond:
                        self.toGameState(GameState.WaitForEngine);

        return ;
        
def main():

    root = Tk();

    # create the application
    app = App(root)

    #
    # here are method calls to the window manager class
    #
    app.master.title('Cloudict.Connect6')
    # app.master.maxsize(840, 840)

    # start the program
    app.mainloop()

    # root.destroy();

if __name__ == '__main__':
    main();
