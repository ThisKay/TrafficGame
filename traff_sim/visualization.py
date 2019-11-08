import csv


def writeToCsv(objectsList, name, type_table="cars"):
    with open(name, "w", encoding="utf-8", newline="") as csvfile:
        writer = csv.writer(csvfile)
        if type_table == "cars":
            if len(objectsList) > 0:
                car = objectsList[0]
                if car.getEndTime():
                    writer.writerow(["Original", "Destination", "StartTime", "EndTime", "TravelTime", "detour"])
                    for car in objectsList:
                        writer.writerow(
                            [car.getOriginal().getPosition(), car.getDestination(), car.getStartTime(),
                             car.getEndTime(), car.getEndTime() - car.getStartTime(), car.detour])
                else:
                    writer.writerow(
                        ["Original", "Destination", "StartTime", "CurrentPosition", "CurrentVelocity",
                         "DirectionToturn", "detour"])
                    for car in objectsList:
                        writer.writerow(
                            [car.getOriginal().getPosition(), car.getDestination(), car.getStartTime(),
                             car.getPosition(), car.getVelocity(), car.direction_toTurn, car.detour])
        else:
            writer.writerow(["linkPosition1", "linkPosition2", "carNum_positive", "carNum_negative"])
            for link in objectsList:
                if abs(link.getPosition1()[0]) == 3200 \
                        or abs(link.getPosition2()[0]) == 3200 \
                        or abs(link.getPosition1()[1]) == 2600 \
                        or abs(link.getPosition2()[1]) == 2600:
                    pass
                else:
                    writer.writerow([link.getPosition1(), link.getPosition2(), len(link.carOnRoad_positive),
                                     len(link.carOnRoad_negative)])
