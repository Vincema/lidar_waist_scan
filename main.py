import lidars
import constants
import utility
import matplotlib.pyplot as plt
import lidarsDrv as lid


def disp_menu():
    print('')
    print('----MENU----') 
    if drv.areConnected == 0:
        print('1: Connect to lidars')
        print('2: Compute circumference')
    else:
        print('1: Disconnect to lidars')
        print('2: Toggle motors')
        print('3: Launch scan')
        print('4: Compute circumference')
    print('q: Quit')

    
def execute_actions():
    choice = input('    Waiting for input: ')

    if choice == 'q' or choice == 'Q':
        drv.disconnect()
        exit()
    
    if drv.areConnected == 0:
        if choice == '1':
            # Connect to lidar
            drv.connect()
        elif choice == '2':
            # Compute circumference
            compute_algo_circumference()
        else:
            print('        Wrong input!')
    else:
        if choice == '1':
            # Disconnect to lidar
            drv.disconnect()
        elif choice == '2':
            # Toggle motors
            if drv.checkLinkState() == 0:
                drv.toggle_motors()
        elif choice == '3':
            # Data scanning
            if drv.checkLinkState() == 0:
                drv.scan_datas()
        elif choice == '4':
            compute_algo_circumference()
        else:
            print('        Wrong input!')
            
        
def compute_algo_circumference():
    # Read lidars datas
    lidarsSet.read_datas_files()

    # Plot raw datas
    lidarsSet.plot_raw_datas()

    # Compute raw datas into merged and centered datas (and save it into miscDatasStruct}
    lidarsSet.compute_raw_datas()

    # Find the optimal smooth factor for the loess regression
    utility.find_optimal_smooth_factor()

    # Get the distance from the origin and angles values and compute a loess regression
    utility.loess_algorithm()

    # Compute circumference
    utility.compute_circumference()

    # Show the plots
    print('\nClose all figures to continue...\n')
    plt.show()



                  
# Init lidars infos
lidarsSet = lidars.setOfLidars()

# Init lidars
drv = lid.driverLidars()

while True:
    disp_menu()
    execute_actions()





