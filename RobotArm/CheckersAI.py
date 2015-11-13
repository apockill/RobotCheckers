'''
Created on Jul 21, 2011

@author: Davide Aversa
'''

import random

class DraughtsBrain(object):
    '''
    Class AI for Draughts.
    
    Use Min-Max with Alpha-Beta Prune.
    '''

    def __init__(self, weights, horizon, weights_bis=None, ):
        '''
        Constructor
            
        ARGS:
            @param weights: Weights for board Static-Evaluation-Function.
            @param horizon: Max level for the search algorithm. 
            @param weights_bis: Weights for the dark-side AI. 
        '''
        self.weights = weights
        self.horizon = horizon
        
        self.move = 0
        self.board = DBoard()
        self.turn = 'LIGHT'
        
        self.gameover = False
        self.winner = None
        self.nocapturecounter = 0 # Move without a capture.
        
        self.verbose = False
        
        if weights_bis == None :
            self.weights_bis = self.weights
        else :
            self.weights_bis = weights_bis            
    
    def reset(self):
        self.move = 0
        self.board = DBoard()
        self.turn = 'LIGHT'
        self.gameover = False
        self.nocapturecounter = 0
    
    def switch_turn(self):
        '''
        Switch current in-game player.
        '''
        if self.turn == 'LIGHT' :
            self.turn = 'DARK'
        else :
            self.turn = 'LIGHT'
    
    def _switch_player(self, player):
        '''
        Switch player tag.
        
        ARGS:
            @param player: Current player.
        
        RETURN:
            @return: Next Player.
        '''
        if player == 'LIGHT' :
            return 'DARK'
        else :
            return 'LIGHT'
               
    def run_self(self):
        '''
        Execute "selfish" AI vs. AI match.
        '''
        self.gameover = False
        while not self.gameover and self.nocapturecounter < 50 :
            bestmove = self.best_move()

            print bestmove

            if not bestmove:
                self.winner = self._switch_player(self.turn) # No valid move!
                break
            self.apply_move(bestmove)
            if self.verbose : 
                print(self.board)
                print(self.board.board_score(self.weights))
        if not self.gameover : # So, too-much noncapture.
            self.winner = 'DRAW'
        return self.winner
                
    def apply_move(self, action):
        '''
        Apply an action to board.
        
        ARGS:
            @param action: Action that it's going to be executed.
        '''
        self.board.apply(action)
        self.move += 1
        if len(self.board.light_pieces) == 0 :
            self.gameover = True
            self.winner = 'DARK'
        elif len(self.board.dark_pieces) == 0 :
            self.gameover = True
            self.winner = 'LIGHT'
        else :
            self.switch_turn()
            if action.type != 'CAPTURE' :
                self.nocapturecounter += 1
            else :
                self.nocapturecounter = 0        
                
    ########
    ## AI ##
    ########
    
    def best_move(self, **kwargs):
        '''
        Find the next best move according current player state.
        
        This method use the Min-Max algorithm wit Alpha-Beta pruning system
        to minimize the number of explored nodes.
        
        RETURN:
            @return: One of the best move.
        '''
        newBoard = kwargs.get("board", [])
        if len(newBoard) > 0:
            self.board = DBoard(board=newBoard)

        if len(self.board.all_move(self.turn)) == 0:
            self.gameover = True
            self.winner = self._switch_player(self.turn)
            return None
            
        self.path = []
        if self.turn == 'LIGHT' :
            value = self.alphabeta(-float('inf'), float('inf'), self.horizon, self.turn, self.weights)
        else :
            value = self.alphabeta(-float('inf'), float('inf'), self.horizon, self.turn, self.weights_bis)
        
        bestmoves = []
        
        for element in self.path :
            if element[1] == value : # Find path with value equal to best-value.
                bestmoves.append(element[0])
        else :
            if len(bestmoves) == 0 and len(self.path) != 0 : # If path is not empty return first value.
                print("Woops!")
                return self.path[0][0] # WARNING: This code should never be executed.
        
        selected_move = random.choice(bestmoves) # Select randomly a move among the best ones.
        return selected_move
                
    def alphabeta(self, alpha, beta, level, player, weights):
        '''
        THE GLORIOUS ALPHA-BETA ALGORITHM. GLORIFY HIM.
        
        ARGS:
            @param aplha: Current Alpha Value.
            @param beta: Current Beta Value.
            @param level: Current Level.
            @param player: Current Player.
            @param weights: Set of weights to use. TODO: Can remove this?
        
        RETURN    
        '''
        if level == 0 :
            value = self.board.board_score(weights)
            self.path.append((self.board.movelist[self.move], value))
            return value
        if player == 'LIGHT' :
            moves = self.board.all_move(player)
            v = -float('inf')
            for mov in moves :
                self.board.apply(mov)
                v = max(v, self.alphabeta(alpha, beta, level - 1, self._switch_player(player), weights))
                self.board.undo_last()
                if beta <= v :
                    return v
                alpha = max(alpha, v)
            if len(moves) == 0 :
                self.path.append((self.board.movelist[self.move], v))
            return v
        else :
            moves = self.board.all_move(player);
            v = float('inf')
            for mov in moves :
                self.board.apply(mov)
                v = min(v, self.alphabeta(alpha, beta, level - 1, self._switch_player(player), weights))
                self.board.undo_last()
                if v <= alpha :
                    return v
                beta = min(beta, v)
            if len(moves) == 0 :
                self.path.append((self.board.movelist[self.move], v))
            return v

class DBoard(object):
    '''
    Class for Draughts board.
    '''

    def __init__(self, **kwargs):
        '''
        Constructor. Initialize an empty draughts board with
        the pieces in starting position.
        PresetBoard should come in this format:
            [[x,x,x,x,x,x]
             [x,x,x,x,x,x]
             [x,x,x,x,x,x]
             [x,x,x,x,x,x]
             [x,x,x,x,x,x]
             [x,x,x,x,x,x]]
        '''

        presetBoard = kwargs.get("board", [])

        self.light_pieces = []
        self.dark_pieces = []
        self.bitmap = [None] * 18  # None = empty
        self.movelist = []

        self.boardSize = 6

        if len(presetBoard) > 0:  #If the AI is being started with a preset board position
            #board = [item for sublist in presetBoard for item in sublist]
            for row in range(len(presetBoard)):
                for column in range(len(presetBoard)):
                    #SET NORMAL PIECE
                    if presetBoard[row][column] == 1:
                        new_piece = DPiece(self, row, column, 'LIGHT')
                        self.light_pieces.append(new_piece)
                        self.set_bitmap(row, column, new_piece)
                    if presetBoard[row][column] == 2:
                        new_piece = DPiece(self, row, column, 'DARK')
                        self.dark_pieces.append(new_piece)
                        self.set_bitmap(row, column, new_piece)

                    #SET KINGS
                    if presetBoard[row][column] == 3:
                        new_piece = DPiece(self, row, column, 'LIGHT', isKing=True)
                        self.light_pieces.append(new_piece)
                        self.set_bitmap(row, column, new_piece)
                    if presetBoard[row][column] == 4:
                        new_piece = DPiece(self, row, column, 'DARK', isKing=True)
                        self.dark_pieces.append(new_piece)
                        self.set_bitmap(row, column, new_piece)


                print presetBoard[row]
        else:
            # Add 6 Dark Piece in starting position.
            row = 0
            column = 1
            delta = 1
            for _ in xrange(6):
                #print "Placing Dark:\trow: ", row, " column:", column
                new_piece = DPiece(self, row, column, 'DARK')
                self.dark_pieces.append(new_piece)
                self.set_bitmap(row, column, new_piece)
                column += 2
                if column > self.boardSize - 1:
                    column -= (self.boardSize + delta)
                    row += 1
                    delta = -delta

            # Add 6 Light Piece in starting position.
            row = 4
            column = 1
            delta = 1
            for _ in xrange(6):
                #print "Placing Light:\trow: ", row, " column:", column
                new_piece = DPiece(self, row, column, 'LIGHT')
                self.light_pieces.append(new_piece)
                self.set_bitmap(row, column, new_piece)
                column += 2
                if column > self.boardSize - 1:
                    column -= (self.boardSize + delta)
                    row += 1
                    delta = -delta

    def __cord2idx(self, row, column):
        '''
        Transform coordinate of the black-square in board into
        index in bitmap map.

        ARGS:
            @param row: Row.
            @param column: Column.

        RETURNS:
            @return: Index of <row,column> square in bitmap.
        '''
        # If row%2 == 0 (column-1)/2 else column/2
        # This should be equivalent...
        #return 5 * row + column / 2
        return (self.boardSize / 2) * row + column / 2  #TODO: Make this change with different board size variables

    def set_bitmap(self, row, column, value):
        '''
        Set bitmap to value.

        ARGS:
         @param row: Row.
         @param column: Column.
         @param value: New value of bitmap.
        '''
        self.bitmap[self.__cord2idx(row, column)] = value

    def is_free(self, row, column):
        '''
        Check if <row,column> square is empty.

        ARGS:
            @param row: Row.
            @param column: Column.

        RETURN:
            @return: True if square is free and on board.
        '''
        if row < 0 or row >= self.boardSize :
            return False
        if column < 0 or column >= self.boardSize :
            return False

        #print self.__cord2idx(row, column)
        if self.bitmap[self.__cord2idx(row, column)] :
            return False

        return True

    def get_piece(self, row, column):
        '''
        Get piece in <row,column> square if any.

        ARGS:
            @param row: Row.
            @param column: Column.

        RETURN:
            @return: Reference to Piece in <row,column>
        '''
        return self.bitmap[self.__cord2idx(row, column)]

    def apply(self, action, chain=False):
        '''
        Apply an action to the board.

        Note that in Chain-Capture *only* the first step must be recorded
        in board undo-stack. So use the `chain` flag to apply chain next
        steps.

        ARGS:
            @param action: Action to apply.
            @param chain: True if action is one step of a chain action.
        '''
        # If ACTION is UNDO-type DO NOT add to undo-list.
        if action.type != 'UNDO' and not chain :
            self.movelist.append(action)

        # Get Source and Destination.
        srow, scol = action.source
        drow, dcol = action.destination

        # Get Piece in Source.
        piece = self.get_piece(srow, scol)

        if piece == None : #If no piece is in source->ERROR.
            raise Exception("NO PIECE IN SOURCE!")

        piece.move(drow, dcol) #Move piece in destination.

        # Check Promotion: Promote only if `action` is a final-step.
        if action.promote and not action.next :
            piece.promote()

        if action.type == 'CAPTURE' or action.type == 'CHAIN' :
            # If action is CAPTURE-type get captured piece.
            captured = action.captured
            captured.captured() #Remove captured piece from board.
            if captured.color == 'LIGHT' : #... and from right list.
                self.light_pieces.remove(captured)
            else :
                self.dark_pieces.remove(captured)
        elif action.type == 'UNDO' :
            # If action in UNDO-type
            captured = action.captured
            if captured != None : # if exist captured piece then re-add.
                self.set_bitmap(captured.position[0], captured.position[1], captured)
                if captured.color == 'LIGHT' :
                    self.light_pieces.append(captured)
                else :
                    self.dark_pieces.append(captured)
            # Demote check! If source is a promote location and is_king
            # then piece must be demoted.
            if action.promote :
                piece.demote()

        # If chain-capture (or chain-undo) apply next step.
        if action.next :
            self.apply(action.next, chain=True) # Record only the first step.

    def all_move(self, color):
        '''
        Get all possible move for a player

        ARGS:
            @param color: Player color.

        RETURN:
            @return: List of all possible action.
        '''
        move = []
        capture = False
        if color == 'LIGHT' :
            for piece in self.light_pieces :
                move = move + piece.possible_action()
        else :
            for piece in self.dark_pieces :
                move = move + piece.possible_action()

        # Check if `move` contains a CAPTURE-action.
        for m in move :
            if m.type == 'CAPTURE' :
                capture = True
                break

        # If this action exist then NO OTHER ACTIONS are allowed.
        # So we remove all action that are not CAPTURE-type.
        if capture :
            move_new = []
            for m in move :
                if m.type == 'CAPTURE' :
                    move_new.append(m)
            return move_new
        else :
            return move

    def board_score(self, weights):
        '''
        Static Evaluation Function for Draughts Board.

        ARGS:
            @param weights: Dictionary of Weights for each feature.

        RETURN:
            @return: The board score.
        '''
        ## Avoid Frequent Look-Up
        ##
        get_features = DPiece.get_features
        ##
        vlight = {'PIECE':   0,
                  'KING':    0,
                  'BACK':    0,
                  'KBACK':   0,
                  'CENTER':  0,
                  'KCENTER': 0,
                  'FRONT':   0,
                  'KFRONT':  0,
                  'MOB':     0}

        vdark = vlight.copy()

        # For each piece, for each feature add the total counter.
        for piece in self.light_pieces :
            features = get_features(piece)
            for f in features :
                vlight[f] += 1

        for piece in self.dark_pieces :
            features = get_features(piece)
            for f in features :
                vdark[f] += 1

        #print weights
        score_light = sum([vlight[key] * weights[key] for key in weights.iterkeys()])
        score_dark  = sum([vdark[key]  * weights[key] for key in weights.iterkeys()])

        return score_light - score_dark # Return difference.

    def undo_last(self):
        '''
        Undo last action.
        '''
        # Get Last Action.
        last = self.movelist.pop()
        undo = last.undo() # Make undo action from this.
        self.apply(undo) # Apply Undo.

    def __str__(self):
        string = ""
        for row in xrange(10) :
            for column in xrange(10) :
                if ((row % 2 == 0) != (column % 2 == 0)) :
                    idx = self.__cord2idx(row, column)
                    if self.bitmap[idx] == None :
                        string += '.'
                    elif self.bitmap[idx].color == 'DARK' :
                        if self.bitmap[idx].is_king :
                            string += '#'
                        else :
                            string += 'X'
                    else :
                        if self.bitmap[idx].is_king :
                            string += '$'
                        else :
                            string += 'O'
                else :
                    string += '.'
            string += '\n'
        return string

class DPiece(object):
    '''
    This class represent a Draughts Piece.
    '''

    def __init__(self, board, row, column, color, **kwargs):
        '''
        Constructor

        @param board: Board in which this piece exists.
        @param row: Starting Row.
        @param column: Starting Column.
        @param color: Piece color (LIGHT or DARK).
        '''
        self.is_king = kwargs.get('isKing', False)
        self.boardSize = 6
        self.startingRows = 2
        self.board = board
        self.position = (row, column)
        self.color = color

    def promote(self):
        '''
        Promote a Piece.
        '''
        self.is_king = True

    def demote(self):
        '''
        Promote a Piece.
        '''
        self.is_king = False

    def get_features(self):
        '''
        Get Features List. See DBoard total score for all Features List.
        '''
        features_list = []
        color = self.color
        row, column = self.position
        if not self.is_king :
            features_list = ['PIECE']
            if row < self.startingRows + 1:
                if color == 'LIGHT':
                    features_list = ['PIECE', 'FRONT']
                else :
                    features_list = ['PIECE', 'BACK']
            if row >= self.boardSize - self.startingRows:
                if color == 'LIGHT':
                    features_list = ['PIECE', 'BACK']
                else :
                    features_list = ['PIECE', 'FRONT']
            if (1 <= row < self.boardSize - 1) and (1 <= column < self.boardSize - 1):
                features_list.append('CENTER')
        else :
            features_list = ['KING']
            if row < self.startingRows + 1:
                if color == 'LIGHT' :
                    features_list = ['KING', 'KFRONT']
                else :
                    features_list = ['KING', 'KBACK']
            if row >= self.boardSize - self.startingRows :
                if color == 'LIGHT':
                    features_list = ['KING', 'KBACK']
                else :
                    features_list = ['KING', 'KFRONT']
            if (1 <= row < self.boardSize - 1) and (1 <= column < self.boardSize - 1):
                features_list.append('KCENTER')
        return features_list

    def move(self, nrow, ncolumn):
        '''
        Move this piece.

        This method DO NOT perform any move control so, please, use
        valid move.

        ARGS:
            @param nrow: Destination Row
            @param ncolumn: Destination Column
        '''
        new_position = (nrow, ncolumn)
        self.board.set_bitmap(self.position[0], self.position[1], None) #Update Bitmap
        self.board.set_bitmap(nrow, ncolumn, self) #TODO: Is better update this on Dboard?
        self.position = new_position

    def captured(self):
        '''
        If a piece is captured by another piece then this one must disappear from
        board bitmap.
        '''
        self.board.set_bitmap(self.position[0], self.position[1], None)

    def _check_promote(self, drow):
        '''
        Check if, in one action, piece become King.
        '''
        if not self.is_king :
            if (self.color == 'LIGHT' and drow == 0) or (self.color == 'DARK' and drow == self.boardSize-1) :
                return True
        return False

    def _possible_action_piece(self):
        '''
        Check for piece possible actions.
        '''
        ## Frequent Look-Up Avoiding
        ##
        is_free = self.board.is_free
        board = self.board
        ##

        move = []
        row, col = self.position
        capture = False # True if piece can capture enemy piece.
        if self.color == 'LIGHT' :
            dr = -1 #Move UP
        else :
            dr = 1  #Move DOWN

        for dc in (-1, 1) :
            if is_free(row + dr, col + dc) :
                if not capture :
                    prom = self._check_promote(row + dr)
                    move.append(DAction('MOVE', (row, col), (row + dr, col + dc), promote=prom))
            elif is_free(row + 2 * dr, col + 2 * dc) :
                obstruction = board.get_piece(row + dr, col + dc)
                if obstruction.color != self.color :
                    prom = self._check_promote(row + 2 * dr)
                    move.append(DAction('CAPTURE', (row, col), (row + 2 * dr, col + 2 * dc), obstruction, prom))
                    capture = True
        if capture :
            move_new = []
            for m in move :
                if m.type == 'CAPTURE' :
                    # Check for chain captures.
                    board.apply(m)
                    next_steps = self.possible_action()
                    board.undo_last()
                    if next_steps and next_steps[0].type == 'CAPTURE' :
                        for step in next_steps :
                            tmp = m.copy()
                            tmp.next = step
                            move_new.append(tmp)
                    else :
                        move_new.append(m)
            return move_new
        else :
            return move

    def _possible_action_king(self):
        '''
        Check King possible actions.
        '''
        ## Frequent Look-Up Avoiding
        ##
        is_free = self.board.is_free
        board = self.board
        ##

        move = []
        row, col = self.position
        capture = False
        direction = ((1, 1), (1, -1), (-1, -1), (-1, 1))
        for dir in direction :
            trow = row + dir[0]
            tcol = col + dir[1]
            while is_free(trow, tcol) :
                if not capture :
                    move.append(DAction('MOVE', (row, col), (trow, tcol)))
                trow += dir[0]
                tcol += dir[1]
            if board.is_free(trow + dir[0], tcol + dir[1]) :
                obstruction = board.get_piece(trow, tcol)
                if obstruction.color != self.color :
                    move.append(DAction('CAPTURE', (row, col), (trow + dir[0], tcol + dir[1]), obstruction))
                    capture = True
        if capture :
            move_new = []
            for m in move :
                if m.type == 'CAPTURE' :
                    # Check for chain captures.
                    board.apply(m)
                    next_steps = self.possible_action()
                    board.undo_last()
                    if next_steps and next_steps[0].type == 'CAPTURE' :
                        for step in next_steps :
                            tmp = m.copy()
                            tmp.next = step
                            move_new.append(tmp)
                    else :
                        move_new.append(m)
            return move_new
        return move

    def possible_action(self):
        '''
        List all possible action for the piece.
        '''
        if self.is_king :
            return self._possible_action_king()
        else :
            return self._possible_action_piece()

class DAction(object):
    '''
    This Class represent an Action on the Draughts Board.

    Exists three type of action:
        * MOVE : Standard Move
        * CAPTURE : Capture Enemy Piece - Can be a Chain Capture.
        * UNDO : Undo Move
    '''

    def __init__(self, type, source, destination, captured=None, promote=False, **kwargs):
        '''
        Constructor

        ARGS:
            @param type: Action Type
            @param source: Tuple (row,column) of starting position.
            @param destination: Tuple (row,column) of ending position.
            @param captured: Captured piece (if type is CAPTURE).
        '''
        self.capturedPiece = kwargs.get('capuredPiece', None)
        self.type = type
        self.source = source
        self.destination = destination
        self.captured = captured
        self.promote = promote
        self.next = None # Next Capture if `CAPTURE` is a Chain-Capture.


    def _append_capture(self, action):
        '''
        Append an item in Chain-Captures at the end of chain.

        ARGS:
            @param action: Action to append.
        '''
        p = self
        while p.next :
            p = p.next
        p.next = action



    def getSource(self):
        return list(self.source)[::-1]

    def getDestination(self):
        return list(self.destination)[::-1]

    def getCapture(self):
        if self.captured is None:
            return None
        else:
            return list(self.captured.position)[::-1]



    def undo(self):
        '''
        Create Undo Action from current Action.

        RETURN:
            @return: Undo Action
        '''
        raw_undo = DAction('UNDO', self.destination, self.source, self.captured, self.promote)

        if self.next == None : # Last element.
            return raw_undo

        undo_rest = self.next.undo() # Invert chain tail.
        undo_this = raw_undo # Invert current step.
        undo_rest._append_capture(undo_this)
        return undo_rest

    def copy(self):
        return DAction(self.type, self.source, self.destination, self.captured, self.promote)

    def __len__(self):
        if self.next :
            return 1 + len(self.next)
        return 1

    def __eq__(self, other):
        if other == None :
            return False
        if self.type != other.type :
            return False
        if self.source != other.source :
            return False
        if self.destination != other.destination :
            return False
        if self.captured != other.captured :
            return False
        return True

    def __repr__(self):
        return self.__str__()

    def __str__(self):
        return "%s :: <%d , %d> -> <%d , %d> { %s }" % (self.type, self.source[0],
                                                        self.source[1], self.destination[0],
                                                        self.destination[1],
                                                        str(self.next))
        #return str([list(self.source), list(self.destination)], list(self.next))


if '__main__' == __name__:
    wLight = {'PIECE':    10,
              'KING':    100,
              'BACK':    5,
              'KBACK':   5,
              'CENTER':  6,
              'KCENTER': 7,
              'FRONT':   5,
              'KFRONT':  5,
              'MOB':     6}

    wDark = {'PIECE':    1,
              'KING':    0,
              'BACK':    0,
              'KBACK':   0,
              'CENTER':  1,
              'KCENTER': 0,
              'FRONT':   0,
              'KFRONT':  0,
              'MOB':     0}
    ai = DraughtsBrain(wLight, 5, wDark)
    ai.run_self()
    #print ai.best_move()
    #ai.board.apply("<4 , 5> -> <3 , 4>")#"<1 , 0> -> <2 , 1>")
    #print ai.best_move()