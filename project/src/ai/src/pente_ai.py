import sys
import os
import argparse
import math

# TODO:
# 1. *Implement minimax agent
# 2. *Add everything else needed for playing game through terminal
# 3. *Implement alpha-beta pruning
# 4. Memoize findRows,Cols,Diags and # pieces
# 5. Add minimax search heuristic
# 6. Refine state evaluation

board_size = [6, 6]     # [WIDTH, HEIGHT] Caution: PenteState will break if not NxN
grid_labels = [
    [chr(j+65) for j in range(board_size[0])],
    [i+1 for i in range(board_size[1])]
]
piece = {
    'EMPTY': 0,
    'WHITE': 1,
    'BLACK': -1,
}


class PenteState(object):
    initCaptured = {'WHITE':0, 'BLACK':0}

    def __init__(self, captured=initCaptured, data=None, dataDict=None, whitesMove=True):
        if data == None and dataDict == None:
            self.data = [[piece['EMPTY'] for _ in range(board_size[0])] for _ in range(board_size[1])]
        elif dataDict == None:
            self.data = list(data)
            for j in range(len(data[0])): self.data[j] = list(data[j])
        elif data == None:
            self.data = [[piece['EMPTY'] for _ in range(board_size[0])] for _ in range(board_size[1])]
            for j, i in dataDict:
                self[j, i] =  dataDict[(j, i)]
        else:
            raise ValueError("Missing/Illegal arguments for PenteState.")

        self.captured = dict(captured)
        self.whitesMove = whitesMove

    def __str__(self):
        state_str = ["Pente Game State:"]
        state_str += ["  ".join(["  "]+[j for j in grid_labels[0]])]

        for i in reversed(grid_labels[1]):
            state_str += ["".join([str(i)+"  "] + 
                                  [" . " if self[j, i] == piece['EMPTY'] else 
                                   " O " if self[j, i] == piece['WHITE'] else
                                   " X "
                                   for j in grid_labels[0]]
                                 )]

        state_str += ["# WHITE PIECES CAPTURED: "+str(self.captured['WHITE'])]
        state_str += ["# BLACK PIECES CAPTURED: "+str(self.captured['BLACK'])]
        state_str += ["NEXT MOVE: WHITE" if self.whitesMove else "NEXT MOVE: BLACK"]
        return "\n".join(state_str)

    def __repr__(self):
        return str(self)

    def __getitem__(self, loc):
        # Array coordinates, eg: (2, 0)
        if type(loc[0]) == int and type(loc[1]) == int:
            return self.data[loc[1]][loc[0]]
        # Board coordinates, eg: ('A', 3)
        elif type(loc[0]) == str and type(loc[1]) == int:
            return self.data[ord(loc[0])-65][loc[1]-1]
        else:
            raise ValueError("Illegal getitem call: "+str(loc))

    def __setitem__(self, loc, val):
        # Avoid using this, states should be immutable
        # Array coordinates, eg: (2, 0)
        if type(loc[0]) == int and type(loc[1]) == int:
            self.data[loc[1]][loc[0]] = val
        # Board coordinates, eg: ('A', 3)
        elif type(loc[0]) == str and type(loc[1]) == int:
            self.data[ord(loc[0])-65][loc[1]-1] = val
        else:
            raise ValueError("Illegal setitem call: "+str(loc))

    def getNumCaptured(self, forWhite):
        return self.captured['WHITE'] if forWhite else self.captured['BLACK']

    def generateSuccessor(self, action):
        assert self[action[0], action[1]] == piece['EMPTY'], \
               "Illegal Successor State attempted: "+str(action)

        newState = PenteState(captured=self.captured, 
                              data=self.data, 
                              whitesMove=not self.whitesMove)
        if self.whitesMove:
            newState[action[0], action[1]] = piece['WHITE'] 
        else:
            newState[action[0], action[1]] = piece['BLACK']
        return newState

    def findRowsAndColumns(self, forWhite):
        color = piece['WHITE'] if forWhite else piece['BLACK']
        rows = [0 for _ in range(board_size[0])]
        cols = [0 for _ in range(board_size[0])]
        for i in range(board_size[1]):
            nr, nc = 0, 0
            for j in range(board_size[0]):
                if self[i,j] == color:
                    nr += 1
                else:
                    rows[nr] += 1
                    nr = 0

                if self[j,i] == color:
                    nc += 1
                else:
                    cols[nc] += 1
                    nc = 0
            rows[nr] += 1
            cols[nc] += 1

        # Only rows count single runs during score evaluation
        return [rows[n]+cols[n] for n in range(2,board_size[0])]

    def findDiagonals(self, forWhite):
        # Utelizes coordinate transform for iterating over diagonals:
        # <row, col> --> <diagonal base row, diagonal magnitude> :: <i,j> --> <p,q>
        # http://stackoverflow.com/a/6313407
        color = piece['WHITE'] if forWhite else piece['BLACK']
        forwards = [0 for _ in range(board_size[0])]
        backwards = [0 for _ in range(board_size[0])]
        N = board_size[0]
        for p in range(2*N-1):
            nf, nb = 0, 0
            for q in range(max(0, p-N+1), min(p+1, N)):
                if self[q, p-q] == color:
                    nf += 1
                else:
                    forwards[nf] += 1
                    nf = 0

                if self[N-1-q, p-q] == color:
                    nb += 1
                else:
                    backwards[nb] += 1
                    nb = 0
            forwards[nf] += 1
            backwards[nb] += 1

        return [forwards[n] + backwards[n] for n in range(2,board_size[0])]

    def isLegalAction(self, action):
        return self.getGridValue(action[0], action[1]) == piece['EMPTY']

    def getLegalActions(self):
        # Dunno if this is worth implementing
        return [(i, j) for j in range(board_size[0])
                       for i in range(board_size[1])
                       if self[i,j] == piece['EMPTY']]

    def isLose(self, forWhite):
        runs = self.findRowsAndColumns(not forWhite) + self.findDiagonals(not forWhite)
        for l in range(5,board_size[0]):
            if l in runs: return True
        return False

    def isWin(self, forWhite):
        runs = self.findRowsAndColumns(forWhite) + self.findDiagonals(forWhite)
        for l in range(5,board_size[0]):
            if l in runs: return True
        return False


class PenteAgent(object):
    def __init__(self, color, depth=2):
        self.depth = depth
        self.color = True if color == 'WHITE' else False

    def evaluationFunction(self, gameState):
        def length_to_score(l):
            return math.exp(l) if l < 5 else 4294967295.
        playerRuns = gameState.findRowsAndColumns(self.color) + gameState.findDiagonals(self.color)
        opponentRuns = gameState.findRowsAndColumns(not self.color) + gameState.findDiagonals(not self.color)
        score = 0
        
        # There should never be normal game where both sides achieve a row of 5
        # simultaneously. However, there may be a clever solution for that too.
        for l in playerRuns:
            score += length_to_score(l)*playerRuns[l]
        for l in opponentRuns:
            score -= length_to_score(l)*opponentRuns[l]

        return score

    def getAction(self, gameState):
        legalActions = gameState.getLegalActions()
        # Don't bother searching minimax tree for first move
        if len(legalActions) == board_size[0]*board_size[1]: return (4,4)

        actionList = []
        self.beta = float('inf')
        self.alpha = -float('inf')
        for action in legalActions:
            score = self.minimaxer(gameState.generateSuccessor(action),
                                   not self.color,
                                   self.depth)
            actionList.append((score, action))
        return max(actionList)[1]

    def minimaxer(self, gameState, color, depth):
        nextColor = not color
        nextDepth = depth if nextColor != self.color else depth-1
        if depth == 0 or gameState.isLose(self.color) or gameState.isWin(self.color):
            score = self.evaluationFunction(gameState)
            return score

        minScore = float('inf')
        maxScore = -float('inf')
        for action in gameState.getLegalActions():
            actionScore = self.minimaxer(gameState.generateSuccessor(action),
                                         nextColor,
                                         nextDepth)
            if color == self.color:
                maxScore = max(actionScore, maxScore)
                if maxScore >= self.beta: return maxScore
                self.alpha = max(self.alpha, maxScore)
            else:
                minScore = min(actionScore, minScore)
                if minScore <= self.alpha: return minScore
                self.beta = min(self.beta, minScore)

        return maxScore if color == self.color else minScore



class PenteGame(object):
    def __init__(self):
        pass

def main():
    s = PenteState()
    s[2,2] = piece['BLACK']
    a = PenteAgent('WHITE')

    while True:
        print(s)
        act = a.getAction(s)
        print("")
        print("AI takes: "+str(act))
        print("")
        s = s.generateSuccessor(act)
        print(s)

        break
        # act = raw_input("--> ")
        # s = s.generateSuccessor(eval(act))

if __name__ == '__main__':
    main()