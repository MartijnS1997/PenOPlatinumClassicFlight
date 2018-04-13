package internal.Exceptions;

import internal.Physics.WingPhysX;

/**
 * Created by Martijn on 18/10/2017.
 * @author Martijn Sauwens
 */
public class AngleOfAttackException extends IllegalStateException {

    /**
     * Constructor for the angle of attack exception
     * @param causeMessage the message to specify the cause
     * @param causeWing the wing that caused the exception
     */
    public AngleOfAttackException(String causeMessage, WingPhysX causeWing){
        this.causeMessage = causeMessage;
        this.causeWing = causeWing;
    }

    /**
     * Constructor for the angle of attack exception
     * @param causeMessage a message to specify the cause
     */
    public AngleOfAttackException(String causeMessage){
        this(causeMessage, null);
    }

    /**
     * Constructor for the angle of attack exception
     * @param causeWing the wing that caused the exception
     */
    public AngleOfAttackException(WingPhysX causeWing){
        this(null, causeWing);
    }

    /**
     * Getter for the cause message
     * @return a string containing the error message
     */
    public String getCauseMessage(){
        return this.causeMessage;
    }

    /**
     * Getter for the wing that caused the exception
     * @return a wing object containing the wing that caused the exception
     */
    public WingPhysX getCauseWing(){
        return this.causeWing;
    }


    /**
     * Variable that stores the wing that caused the exception
     */
    private WingPhysX causeWing;

    /**
     * variable that contains the message of the exception
     */
    private String causeMessage;

    @Override
    public String toString() {
        return TAG;
    }

    /**
     * Tag used for error examination
     */
    public final static String TAG = "AngleOfAttackException";

}
