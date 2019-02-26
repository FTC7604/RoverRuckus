package org.firstinspires.ftc.teamcode;

//class for the toggle switch.
public class Toggle {
    //toggle is duh, loop is the previous loop
    private boolean toggle;
    private boolean pastLoop;
    private boolean currentLoop;

    //creates the toggle as either any value
    Toggle(boolean start){toggle = start;}
    Toggle(){toggle = false;}

    //returns the value of the toggle
    public boolean get(){ return toggle;}
    //indicates wether or not the switch has toggled
    public boolean changed(){
        if(currentLoop != pastLoop)return true;
        else return false;
    }
    //changes the toggle based on the new data
    public void update(boolean currentLoop){
        this.currentLoop = currentLoop;

        if(changed())toggle = this.currentLoop;
        pastLoop = this.currentLoop;
    }
}
