import lidars
import utility
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import lidarsDrv as lid
import pylab


def disp_menu():
    print('')
    print('----MENU----') 
    if drv.areConnected == 0:
        print('1: Connect to lidars')
        print('2: Plot datas')
        print('3: Compute circumference')
    else:
        print('1: Disconnect to lidars')
        print('2: Start motors')
        print('3: Stop motors')
        print('4: Launch scan')
        print('5: Plot datas')
        print('6: Compute circumference')
    print('q: Quit')

    
def execute_actions():
    choice = input('    Waiting for input: ')

    if choice == 'q' or choice == 'Q':
        if drv.areConnected == 1:
            drv.disconnect()
        return True
    
    if drv.areConnected == 0:
        if choice == '1':
            # Connect to lidar
            drv.connect()
        elif choice == '2':
            lidarsSet.read_datas_files()
            lidarsSet.plot_raw_datas()
            print('\nClose all figures to continue...\n')
            plt.show(block=True)
        elif choice == '3':
            # Compute circumference
            compute_algo_circumference()
        else:
            print('        Wrong input!')
    else:
        if choice == '1':
            # Disconnect to lidar
            drv.disconnect()
        elif choice == '2':
            if drv.checkLinkState() == 0:
                drv.start_motors()        
        elif choice == '3':
            if drv.checkLinkState() == 0:
                drv.stop_motors()       
        elif choice == '4':
            # Data scanning
            if drv.checkLinkState() == 0:
                drv.scan_datas()
        elif choice == '5':
            lidarsSet.read_datas_files()
            lidarsSet.plot_raw_datas()
            print('\nClose all figures to continue...\n')
            plt.show(block=True)
        elif choice == '6':
            compute_algo_circumference()
        else:
            print('        Wrong input!')
    return False            
        
        
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
    plt.show(block=True)

   
# Init lidars infos
lidarsSet = lidars.setOfLidars()

# Init lidars
drv = lid.driverLidars()

end = False
while end == False:
    disp_menu()
    end = execute_actions()

print("Program ended")



