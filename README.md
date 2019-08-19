# Installation Guide


- The target platform for this source code is the nrF552840DK and it is compatible with the PCA10056 Development board and s140 softdevice. This implies that for each application example
in the source code, the file path must possess the following attributes: 
`examples/name_of_application_category/application/pca10056/s140/<ses if you use segger studio>`

- The BLE application was programmed with the  SEGGER Embedded Studio though other IDEs can be used as specified by Nordic Semiconductors.

- Clone or download the source code.

+ [Download](https://www.segger.com/downloads/embedded-studio/) and install the most recent releases of SEGGER Embedded Studio (SES) and the [J-Link](https://www.segger.com/downloads/jlink/) Software and Documentation Pack.
   + Download the software packages for your operating system from SEGGER downloads.
   + You need the following packages:
      + Embedded Studio for ARM (version 3.30 or later)
      + J-Link Software and Documentation Pack (version 6.10g or later)
      
      
**Note:** Compilers tend to run into problems with long path names. Therefore, place the downloaded source code as close to the root level of your file system as possible 
          (for example, at C:/Nordic/SDK). Also, avoid using spaces in the file path and folder name.