Fusion Descriptor - Getting Started
=================

Installation & Running
-----------------

- Download the repository as .zip or run `git clone https://github.com/cadop/fusion360descriptor.git` in Git Bash to download the repository.
- Navigate to the **Utilities** tab in Fusion 360 and click on **Scripts and Add-Ins**.
- Click on the green + to add the script into Fusion 360.
- Add the **Descriptor** folder from the zip file to the scripts. You may have to unzip the file after downloading from Github
- Click on **Descriptor** then click on **Run**. The GUI will appear.
- Rather than configuring the export manually in the GUI, you can create a YAML configuration file to control the export. A number of more
  advanced features are only available via the configuraiton file. See `configuration_sample.yaml` for more information.


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

**A:** Make sure you have Python installed, Microsoft Visual Studio C++ build tools installed, and the ability to use pip commands. In the terminal, run the command `pip install pybullet`

**Q:** PyBullet is not launching, what do I do?

**A:** Make sure that the above 3 things are ready on your computer. Open *hello_bullet.py* from the folder added in your save directory in VSCode and run the program.

**Q:** How can I create more advanced models?

**A:** A good place to find pre-built components is the *McMASTERR-CARR* component directory. To go to this, click on **Insert** in the top menu bar, and in the drop down menu, go to **Insert McMASTERR-Carr Component**. Then, you can find your desired component, click on the specifications of choice, go to **Product Detail**, and press **Download**.
