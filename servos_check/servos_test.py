from pyax12.connection import Connection

def disp_menu():
    global servos
    
    print("\nAvailable servos: ",end="")
    servosIDs = []
    for i in range(254):
        if servos.ping(i):
            servosIDs.append(i)
            if len(servosIDs) == 3:
                break
    for i in range(len(servosIDs)):
        print(servosIDs[i]," ",end="")
    print("")
    
    print('----MENU----') 
    print('1: Set position')
    print('2: Set new ID')
    print('3: Set servo speed')
    print('4: Print control table')
    print('5: Set baudrate 57600')
    print('6: New scan')
    print('7: Set angle limit')
    print('q: Quit')
    
    choice = input('    Waiting for input: ')
    if choice == 'q' or choice == 'Q':
        return True
    
    if choice == '1':
        choice = input('    Select a servo (254 for broadcast): ')
        ID = int(choice)
        if (ID in servosIDs) or ID == 254:
            choice = input('    Position: ')
            position = int(choice)
            if abs(position) <= 150: 
                servos.goto(ID,position,degrees=True)
            else:
                print('    Wrong position (-150 to +150)!')
        else:
            print('    Wrong ID!')
    elif choice == '2':
        choice = input('    Select a servo: ')
        ID = int(choice)
        if ID in servosIDs:
            choice = input('    New ID: ')
            newID = int(choice)
            if newID not in servosIDs and newID >= 1 and newID <= 253:
                servos.set_id(ID,newID)
            else:
                print('    Wrong new ID!')
        else:
            print('    Wrong ID!')
            
    elif choice == '3':
        choice = input('    Speed (1 to 1023): ')
        newSpeed = int(choice)
        if newSpeed >= 1 and newSpeed <= 1023:
            servos.set_speed(254,newSpeed)
        else:
            print('    Wrong speed!')
            
    elif choice == '4':
        choice = input('    Select a servo: ')
        ID = int(choice)
        if ID in servosIDs:
            servos.pretty_print_control_table(ID)
        else:
            print('    Wrong ID!')

    elif choice == '5':
        for i in range(256):
            custom_bd = 2000000 / (i+1)
            servos_custom_br = Connection(port="/dev/ttyAMA0", baudrate=custom_bd, rpi_gpio=True)
            servos_custom_br.set_baud_rate(254,57.600)
        servos = Connection(port="/dev/ttyAMA0", baudrate=57600, rpi_gpio=True)

    elif choice == '6':
        pass

    elif choice == '7':
        choice = input('    Select a servo (254 for broadcast): ')
        ID = int(choice)
        if (ID in servosIDs) or ID == 254:
            choice = input('    CW angle limit: ')
            cw_angle_limit = int(choice)
            if abs(cw_angle_limit) <= 150: 
                servos.set_cw_angle_limit(ID,cw_angle_limit,degrees=True)
            else:
                print('    Wrong angle limit (-150 to 150)!')

            choice = input('    CCW angle limit: ')
            ccw_angle_limit = int(choice)
            if abs(ccw_angle_limit) <= 150: 
                servos.set_ccw_angle_limit(ID,ccw_angle_limit,degrees=True)
            else:
                print('    Wrong angle limit (-150 to 150)!')
            
    else:
        print('    Wrong input!')
    return False
                
            

servos = Connection(port="/dev/ttyAMA0", baudrate=57600, rpi_gpio=True)
end = False
while end == False:
    try:
        end = disp_menu()
    except:
        print("An error has occured!")
        pass
servos.close()
