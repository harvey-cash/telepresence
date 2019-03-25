using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class TileMatcher : MonoBehaviour
{
    public Transform selectObject;

    private const int RANGE_DEG = 120;
    private const int PATTERN_COUNT = 32, TILE_COUNT = 7;

    private float verticalTileOffset = 0.1f;
    private Vector3 tileScale = new Vector3(0.05f, 0.5f, 0.25f);

    private Material[] tileMaterials = new Material[PATTERN_COUNT];
    private GameObject[] tiles = new GameObject[TILE_COUNT];

    private int[] matchingIndices;
    private int chosenIndex;
    private int selection = (int)Mathf.Floor(TILE_COUNT / 2f);
    private bool toggle = true, canChoose = true;

    // Start is called before the first frame update
    void Start()
    {
        selectObject.localScale = tileScale;

        for (int i = 0; i < PATTERN_COUNT; i++) {
            tileMaterials[i] = (Material)Resources.Load("Materials/Tile (" + (i+1) + ")");
        }

        NewRound();
        MoveSelection(0);
    }

    private void NewRound() {
        Material[] roundMaterials = CreateRound();

        for (int i = 0; i < TILE_COUNT; i++) {
            if (tiles[i] != null) { Destroy(tiles[i]); } // Delete previous round

            tiles[i] = CreateTile(i, roundMaterials[i]);
        }
    }

    private GameObject CreateTile(int i, Material material) {
        GameObject tile = GameObject.CreatePrimitive(PrimitiveType.Cube);
        tile.transform.localScale = tileScale;
        tile.transform.forward = Vector3.left;
        tile.transform.parent = this.transform;

        tile.transform.localPosition += Vector3.forward + Vector3.up * (tileScale.y * 0.5f + verticalTileOffset);
        tile.transform.RotateAround(Vector3.zero, Vector3.up, i * RANGE_DEG / (TILE_COUNT - 1) - RANGE_DEG / 2f);

        tile.GetComponent<Renderer>().material = material;

        return tile;
    }

    private Material[] CreateRound() {
        chosenIndex = -1; // Nothing chosen

        // Choose unique materials
        int[] materialIndices = new int[TILE_COUNT];
        Material[] materials = new Material[TILE_COUNT];

        for (int i = 0; i < TILE_COUNT; i++) {
            int newIndex;
            do {
                newIndex = Random.Range(0, PATTERN_COUNT);
            } while (i > 0 && AlreadyChosen(newIndex, materialIndices));
            materialIndices[i] = newIndex;
            materials[i] = tileMaterials[newIndex];
        }
        
        int firstMatching = Random.Range(0, TILE_COUNT); // Choose random to be the matching tile
        int secondMatching;
        do {
            secondMatching = Random.Range(0, TILE_COUNT); // Choose another (unique) random!
        } while (secondMatching == firstMatching);

        materials[secondMatching] = materials[firstMatching]; // Get matching material

        matchingIndices = new int[] { firstMatching, secondMatching };
        return materials;
    }

    // Check index value isn't already chosen
    private bool AlreadyChosen(int val, int[] array) {
        for (int i = 0; i < array.Length; i++) {
            if (array[i] == val) {
                return true;
            }
        }
        return false;
    }

    // Update is called once per frame
    void Update()
    {
        float move = Input.GetAxis("Horizontal");
        if (toggle && Mathf.Abs(move) > 0.5f) {
            MoveSelection(Mathf.RoundToInt(move));

            StartCoroutine(Toggle());
        }

        bool buttonPress = Input.GetKeyDown(KeyCode.Joystick1Button0) || Input.GetKeyDown(KeyCode.Space);
        if (canChoose && buttonPress) {
            Choose(selection);
        }
    }

    private IEnumerator Toggle() {
        toggle = false;
        yield return new WaitForSeconds(0.2f);
        toggle = true;
    }

    private void MoveSelection(int delta) {
        if (chosenIndex != selection) {
            tiles[selection].GetComponent<Renderer>().material.color = new Color(1, 1, 1);
        }        

        selection += delta;
        if (selection < 0) { selection = 0; }
        if (selection > TILE_COUNT - 1) { selection = TILE_COUNT - 1; }

        selectObject.position = tiles[selection].transform.position;
        selectObject.rotation = tiles[selection].transform.rotation;
    }

    private void Choose(int index) {
        // Already chosen
        if (index == chosenIndex) {
            chosenIndex = -1;
            tiles[index].GetComponent<Renderer>().material.color = new Color(1, 1, 1);
        }
        else {
            // Unchosen
            if (chosenIndex == -1) {
                chosenIndex = index;
                tiles[index].GetComponent<Renderer>().material.color = new Color(0.5f, 0.8f, 0.5f);
            }
            else {
                if (chosenIndex == matchingIndices[0] && index == matchingIndices[1]) {
                    tiles[index].GetComponent<Renderer>().material.color = new Color(0.5f, 0.8f, 0.5f);
                    StartCoroutine(Win());
                }
                else if (chosenIndex == matchingIndices[1] && index == matchingIndices[0]) {
                    tiles[index].GetComponent<Renderer>().material.color = new Color(0.5f, 0.8f, 0.5f);
                    StartCoroutine(Win());
                }
                else {
                    Debug.Log("Wrong!");
                    tiles[chosenIndex].GetComponent<Renderer>().material.color = new Color(1, 1, 1);
                    chosenIndex = -1;                    
                }
            }
        }
    }

    private IEnumerator Win() {
        Debug.Log("Win!");
        selectObject.GetComponentInChildren<Renderer>().enabled = false;

        canChoose = false;
        float timeDelay = 2f / (float)TILE_COUNT;
        for (int i = 0; i < TILE_COUNT; i++) {
            yield return new WaitForSeconds(timeDelay);
            tiles[i].GetComponent<Renderer>().enabled = false;
        }
        NewRound();
        yield return new WaitForSeconds(0.5f);
        /*
        for (int i = 0; i < TILE_COUNT; i++) {
            tiles[i].GetComponent<Renderer>().enabled = true;
        }
        */
        selectObject.GetComponentInChildren<Renderer>().enabled = true;
        canChoose = true;
    }
}
