"""VT100 display model, including CR, cursor movement, UTF-8 and scrollback."""
import pyte


class TerminalBuffer:
    def __init__(self,columns=100,rows=30,history=1200):
        self.screen=pyte.HistoryScreen(columns,rows,history=history)
        self.stream=pyte.Stream(self.screen)
        self.dirty=True

    def feed(self,text):
        self.stream.feed(text)
        self.dirty=True

    def reset(self):
        self.screen.reset()
        self.dirty=True

    def resize(self,columns,rows):
        if (columns,rows)!=(self.screen.columns,self.screen.lines):
            self.screen.resize(lines=rows,columns=columns)
            self.dirty=True

    def render(self):
        lines=[]
        for line in self.screen.history.top:
            lines.append(''.join(line.get(i,self.screen.default_char).data for i in range(self.screen.columns)).rstrip())
        history_rows=len(lines)
        lines.extend(self.screen.display)
        self.dirty=False
        # Wide Chinese glyphs occupy two VT cells but one Tk text character.
        row=self.screen.buffer[self.screen.cursor.y]
        prefix=''.join(row.get(i,self.screen.default_char).data for i in range(self.screen.cursor.x))
        return '\n'.join(lines),(history_rows+self.screen.cursor.y+1,len(prefix))
