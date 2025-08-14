using UnityEngine;

public class Wheel : MonoBehaviour {
    
    public bool isTouchingGround;

    void OnTriggerEnter(Collider col) {
        isTouchingGround = true;
    }

    void OnTriggerStay(Collider col) {
        isTouchingGround = true;
    }

    void OnTriggerExit(Collider col) {
        isTouchingGround = false;
    }

}
