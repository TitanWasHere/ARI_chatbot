import sys

def main():
    # Stampa il messaggio "alive" appena avviato
    sys.stdout.write("alive\n")
    sys.stdout.flush()

    contatore = 1

    while True:
        # Legge un messaggio dallo stdin
        messaggio = sys.stdin.readline()
        if not messaggio:
            break
        # Risposta con contatore incrementato
        risposta = f"Ricevuto il messaggio numero {contatore}\n"
        contatore += 1

        # Forza il flush di stdout
        sys.stdout.write(risposta)
        sys.stdout.flush()

if __name__ == "__main__":
    main()
