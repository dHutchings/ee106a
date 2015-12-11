import sys
import os
import argparse
import collections

# TODO:
# 1. Implement minimax agent
# 2. Add everything else needed for playing game through terminal
# 3. Implement alpha-beta pruning
# 4. Refine state evaluation
# 5. Add minimax search heuristic

board_size = [8, 8]     # [WIDTH, HEIGHT] Caution: PenteState will likely break if not NxN
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

    def __init__(self, captured=initCaptured, data=None, dataDict=None, whitesMove=False):
        if data == None and dataDict == None:
            self.data = [[piece['EMPTY'] for _ in range(board_size[0])] for _ in range(board_size[1])]
        elif dataDict == None:
            self.data = data    # Note this is not a protected copy! Will improve runtime when generating new states.
        elif data == None:
            self.data = [[piece['EMPTY'] for _ in range(board_size[0])] for _ in range(board_size[1])]
            for j, i in dataDict:
                self.setGridValue(i-1, ord(j)-65, dataDict[(j, i)])
        else:
            raise ValueError("Missing/Illegal arguments for PenteState.")

        self.captured = dict(captured)
        self.whitesMove = whitesMove

    def __str__(self):
        state_str = ["Pente Game State:"]
        state_str += ["  ".join(["  "]+[j for j in grid_labels[0]])]

        for i in reversed(grid_labels[1]):
            state_str += ["".join([str(i)+"  "] + 
                                  [" . " if self.getGridValue(i-1, j) == piece['EMPTY'] else 
                                   " O " if self.getGridValue(i-1, j) == piece['WHITE'] else
                                   " X "
                                   for j in range(board_size[0])]
                                 )]

        state_str += ["# WHITE PIECES CAPTURED: "+str(self.captured['WHITE'])]
        state_str += ["# BLACK PIECES CAPTURED: "+str(self.captured['BLACK'])]
        state_str += ["NEXT MOVE: WHITE" if self.whitesMove else "NEXT MOVE: BLACK"]
        return "\n".join(state_str)

    def __repr__(self):
        return str(self)

    def getGridValue(self, i, j):
        return self.data[i][j]

    def setGridValue(self, i, j, val):
        # Avoid using this, states should be immutable
        self.data[i][j] = val

    def getNumCaptured(self, forWhite):
        return self.captured['WHITE'] if forWhite else self.captured['BLACK']

    def findRowsAndColumns(self, forWhite):
        color = piece['WHITE'] if forWhite else piece['BLACK']
        rows = collections.Counter()
        cols = collections.Counter()
        for i in range(board_size[1]):
            nr, nc = 0, 0
            for j in range(board_size[0]):
                if self.getGridValue(i, j) == color:
                    nr += 1
                else:
                    rows[nr] += 1
                    nr = 0

                if self.getGridValue(j, i) == color:
                    nc += 1
                else:
                    cols[nc] += 1
                    nc = 0
            rows[nr] += 1
            cols[nc] += 1

        # Only rows count single runs during score evaluation
        del rows[0], cols[0], cols[1]
        return rows, cols

    def findDiagonals(self, forWhite):
        # Utelizes coordinate transform for iterating over diagonals:
        # <row, col> --> <diagonal base row, diagonal magnitude> :: <i,j> --> <p,q>
        # http://stackoverflow.com/a/6313407
        color = piece['WHITE'] if forWhite else piece['BLACK']
        forwards = collections.Counter()
        backwards = collections.Counter()
        N = board_size[0]
        for p in range(2*N-1):
            nf, nb = 0, 0
            for q in range(max(0, p-N+1), min(p, N-1)):
                if self.getGridValue(q, p-q) == color:
                    nf += 1
                else:
                    forwards[nf] += 1
                    nf = 0

                if self.getGridValue(N-1-q, p-q) == color:
                    nb += 1
                else:
                    backwards[nb] += 1
                    nb = 0
            forwards[nf] += 1
            backwards[nb] += 1

        del forwards[0], forwards[1]
        del backwards[0], backwards[1]
        return forwards + backwards

    def getScore(self, forWhite):
        def length_to_score(l):
            return math.exp(l) if l < 5 else 4294967295.
        color = piece['WHITE'] if forWhite else piece['BLACK']
        playerRuns = self.findRowsAndColumns(forWhite) + self.findDiagonals(forWhite)
        opponentRuns = self.findRowsAndColumns(not forWhite) + self.findDiagonals(not forWhite)
        score = 0
        
        # There should never be normal game where both sides achieve a row of 5
        # simultaneously. However, there may be a clever solution for that too.
        for l in playerRuns:
            score += length_to_score(l)*playerRuns[l]
        for l in opponentRuns:
            score -= length_to_score(l)*opponentRuns[l]

        return score

    def isLegalAction(self, move):
        return self.getGridValue(move[0], move[1]) == piece['EMPTY']

    def getLegalActions(self):
        # Dunno if this is worth implementing
        return


class PenteAgent(object):
    def __init__(self):
        pass

class PenteGame(object):
    def __init__(self):
        pass

    def getSuccessorState(self, game_state, move):
        assert game_state.getGridValue(move[0], move[1]) == piece['EMPTY'], \
               "Illegal Successor State attempted: "+str(move)

        newState = PenteState(captured=game_state.captured, 
                              data=game_state.data, 
                              whitesMove=not game_state.whitesMove)
        newState.setGridValue(move[0], move[1], piece['WHITE'] if game_state.whitesMove else piece['BLACK'])
        return newState

def main():
    pass

if __name__ == '__main__':
    main()