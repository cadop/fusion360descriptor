Fusion Descriptor - Getting Started
=================

Creating a Fusion 360 Model
-----------------

**Tutorial**

- Navigate to the **Solid** tab at the top of the screen and click on **Create Sketch**.


<img src="imagesforgettingstarted/1.jpg" alt="drawing" width="400"/>

- Choose a plane to work on by clicking any of the highlighted squares. At the top of your screen, you should now be under the **Sketch** tab.

<img src="imagesforgettingstarted/2.jpg" alt="drawing" width="400"/>

- Click on the **2-Point Rectangle** and drag on the workspace to create a rectangle with a size of your choosing. 

<img src="imagesforgettingstarted/3.jpg" alt="drawing" width="400"/>

- Click on **Finish Sketch**. You should now be back under the **Solid** tab.
- Click on **Extrude** and drag on the rectangle that you have created until you have a 3D shape of your choosing. Click on on **OK**.

<img src="imagesforgettingstarted/4.jpg" alt="drawing" width="400"/>

- Now, you should have a 3D shape categorized under **Bodies** on the left side.

<img src="imagesforgettingstarted/5.jpg" alt="drawing" width="400"/>

- Create a second rectangle following the above instructions that is adjacent to the first one. Make sure they are touching in some way.

<img src="imagesforgettingstarted/6.jpg" alt="drawing" width="400"/>

- You can move a body by right clicking on the body name on the left side and clicking **Move/Copy**, or by pressing **M** on the keyboard.


<img src="imagesforgettingstarted/7.jpg" alt="drawing" width="400"/>

- Right click on **Bodies** and click on *Create Components from Bodies*. Both bodies should now be **Components**.

<img src="imagesforgettingstarted/8.jpg" alt="drawing" width="400"/>

- Make sure that one of these components are grounded. To do so, right click on the component on the left side and click on **Ground**. 
- A red indicator should appear next to the name of the component that shows it is grounded.

<img src="imagesforgettingstarted/9.jpg" alt="drawing" width="400"/>

- Now, go to the **Surface** tab and click on **Joint**.

<img src="imagesforgettingstarted/10.jpg" alt="drawing" width="400"/>

- Add the joint to each component, where they are touching. Make sure this joint has motion type **Rigid**. A **Joints** category should appear on the left side.
- Navigate to the **Utilities** tab at the top of the screen and click on **Scripts and Add-Ins**.

<img src="imagesforgettingstarted/11.jpg" alt="drawing" width="400"/>

- Click on the green + to add the script into Fusion 360.

<img src="imagesforgettingstarted/12.jpg" alt="drawing" width="400"/>

- Add the **Descriptor** folder from the zip file to the scripts.
- Click on **Descriptor** then click on **Run**. The GUI will appear.
- Add a save directory for the output and select the desired options.

<img src="imagesforgettingstarted/13.jpg" alt="drawing" width="400"/>

- Click on **Generate**. The output can be found where the save directory is.

To open the URDF file in PyBullet, do the following:
-----------------

- Find the folder: modelName_description
- Open hello_bullet.py in VSCode or the desired editor and run the program.
- The bullet physics simulator will open and the model will be displayed in the browser.


FAQ
-----------------

**Q:** How do I move things around in Fusion 360?

**A:** You can move a body by clicking on it on the left side and clicking **Move/Copy**, or pressing **M** on the keyboard and then clicking on the desired object.

**Q:** How do I install PyBullet?

**A:** Make sure we have Python installed, Microsoft Visual Studio C++ build tools installed, and the ability to use pip commands. In the Command Prompt, run the command `pip install pybullet`

**Q:** PyBullet is not launching, what do I do?

**A:** Make sure that the above 3 things are ready on your computer. Open *hello_bullet.py* from the folder added in your save directory in VSCode and run the program.

**Q:** How can I create more advanced models?

**A:** A good place to find pre-built components is the *McMASTERR-CARR* component directory. To go to this, click on **Insert** in the top menu bar, and in the drop down menu, go to **Insert McMASTERR-Carr Component**. Then, you can find your desired component, click on the specifications of choice, go to **Product Detail**, and press **Download**.
